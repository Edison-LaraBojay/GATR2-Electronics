// libcamera_camera.cpp
// Capture path: one process-wide CameraManager, one acquired Camera, one
// YUV420 stream at exactly the configured size, four driver buffers each
// bound to one Request and mapped read-only once. libcamera completes
// requests on its own thread; the slot there only hands the Request to the
// capture thread, which copies the Y plane out, requeues the buffer, and
// publishes the copy as the newest frame. The runtime reads that slot under
// a mutex and never sees driver memory.

#include "impl/resources/libcamera_camera.h"

#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/dma-buf.h>
#include <time.h>

#include <cerrno>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <limits>
#include <mutex>
#include <thread>
#include <vector>

#include <libcamera/libcamera.h>

#include "core/host_clock.h"
#include "impl/resources/camera_capture.h"

namespace navigatr
{

namespace
{

#define NAVIGATR_STRINGIFY_INNER(x) #x
#define NAVIGATR_STRINGIFY(x) NAVIGATR_STRINGIFY_INNER(x)

constexpr unsigned int kBufferCount = 4;

std::string errnoText(int rc) { return std::strerror(rc < 0 ? -rc : rc); }

// libcamera allows one CameraManager per process. The first camera starts
// it, the last camera to die stops it; acquire and release share a mutex so
// a new manager is never constructed while the old one is still stopping.
struct ManagerHandle {
    libcamera::CameraManager manager;
    bool                     started = false;

    ~ManagerHandle() {
        if (started) {
            manager.stop();
        }
    }
};

std::mutex& managerMutex() {
    static std::mutex mutex;
    return mutex;
}

std::weak_ptr<ManagerHandle>& managerSlot() {
    static std::weak_ptr<ManagerHandle> slot;
    return slot;
}

std::shared_ptr<ManagerHandle> acquireManager(std::string& err) {
    std::lock_guard<std::mutex>    lock(managerMutex());
    std::shared_ptr<ManagerHandle> handle = managerSlot().lock();
    if (handle != nullptr) {
        return handle;
    }
    handle       = std::make_shared<ManagerHandle>();
    const int rc = handle->manager.start();
    if (rc < 0) {
        err = "CameraManager::start failed: " + errnoText(rc);
        return nullptr;
    }
    handle->started = true;
    managerSlot()   = handle;
    return handle;
}

void releaseManager(std::shared_ptr<ManagerHandle>& handle) {
    std::lock_guard<std::mutex> lock(managerMutex());
    handle.reset();
}

struct MappedPlane {
    void*          base   = nullptr;
    std::size_t    length = 0;
    const uint8_t* y      = nullptr;   // rows of stride bytes
    std::size_t    y_length = 0;
    int            fd = -1;
};

struct Completed {
    libcamera::Request* request = nullptr;
    MonotonicTime       at;   // host clock when libcamera completed it
    CaptureClockSample  clock;
};

// DMA buffers need explicit CPU cache synchronization even when mmap() is
// read-only. Raspberry Pi's rpicam-apps uses the same START/END read pair.
class CpuRead {
public:
    explicit CpuRead(int fd) : fd_(fd), active_(sync(DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ)) {}
    ~CpuRead() { if (active_) { sync(DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ); } }
    bool valid() const { return active_; }
    bool finish() {
        if (!active_) { return false; }
        active_ = false;
        return sync(DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ);
    }
private:
    bool sync(uint64_t flags) {
        dma_buf_sync request{};
        request.flags = flags;
        int rc;
        do { rc = ioctl(fd_, DMA_BUF_IOCTL_SYNC, &request); }
        while (rc < 0 && (errno == EINTR || errno == EAGAIN));
        return rc == 0;
    }
    int fd_;
    bool active_;
};

} // namespace

struct LibcameraCamera::Impl {
    explicit Impl(const CameraCaptureConfig& c) : config(c) {}
    ~Impl() { close(); }

    CameraCaptureConfig config;
    int64_t             configured_frame_us = 0;

    // libcamera objects: set up by open(), touched afterwards only by
    // reset() and close() with the capture thread joined. The capture
    // thread reads camera, stream, stride and mapped, all fixed while it
    // runs, and calls the threadsafe queueRequest.
    std::shared_ptr<ManagerHandle>                   manager;
    std::shared_ptr<libcamera::Camera>               camera;
    bool                                             acquired = false;
    std::unique_ptr<libcamera::CameraConfiguration>  configuration;
    libcamera::Stream*                               stream = nullptr;
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator;
    std::vector<std::unique_ptr<libcamera::Request>> requests;
    std::vector<MappedPlane>                         mapped;   // by request cookie
    libcamera::ControlList                           start_controls;
    bool                                             connected = false;
    bool                                             running   = false;
    unsigned int                                     stride    = 0;
    std::string                                      camera_id;
    std::string                                      stream_note;
    std::string                                      version_note;

    // completion queue: the CameraManager thread pushes, the capture
    // thread pops
    std::mutex              queue_mutex;
    std::condition_variable queue_cv;
    std::deque<Completed>   completed;
    bool                    quit = false;
    std::thread             capture_thread;

    // frame slot and health: the capture thread writes, everyone else
    // reads under the same mutex
    mutable std::mutex             state_mutex;
    std::optional<CameraFrameData> newest;
    bool                           newest_taken          = true;
    uint64_t                       epoch                 = 0;
    uint32_t                       sequence              = 0;
    uint64_t                       frames_captured       = 0;
    uint64_t                       frames_replaced       = 0;
    uint64_t                       frame_errors          = 0;
    uint64_t                       request_sequence_gaps = 0;
    uint64_t                       frame_sequence_gaps   = 0;
    bool                           have_request_sequence = false;
    uint32_t                       last_request_sequence = 0;
    bool                           have_frame_sequence   = false;
    unsigned int                   last_frame_sequence   = 0;
    bool                           opened                = false;
    bool                           started               = false;
    bool                           failed                = false;
    std::string                    last_error;

    bool open(std::string& err);
    bool startCapture(std::string& err);
    void stopCapture();
    void close();
    void fail(const std::string& reason);

    void onRequestCompleted(libcamera::Request* request);   // CameraManager thread
    void onDisconnected();
    void captureLoop();
    void handleCompleted(const Completed& done);
};

bool LibcameraCamera::Impl::open(std::string& err) {
    manager = acquireManager(err);
    if (manager == nullptr) {
        return false;
    }
    version_note = "libcamera " + manager->manager.version() +
                   " (headers " NAVIGATR_STRINGIFY(LIBCAMERA_VERSION_MAJOR) "." NAVIGATR_STRINGIFY(
                       LIBCAMERA_VERSION_MINOR) "." NAVIGATR_STRINGIFY(LIBCAMERA_VERSION_PATCH) ")";

    const std::vector<std::shared_ptr<libcamera::Camera>> cameras = manager->manager.cameras();
    if (config.device_index < 0 ||
        static_cast<std::size_t>(config.device_index) >= cameras.size()) {
        err = "camera index " + std::to_string(config.device_index) +
              " is out of range: libcamera enumerated " + std::to_string(cameras.size()) +
              " camera(s); see rpicam-hello --list-cameras";
        return false;
    }
    camera    = cameras[static_cast<std::size_t>(config.device_index)];
    camera_id = camera->id();

    int rc = camera->acquire();
    if (rc < 0) {
        err = "cannot acquire camera " + camera_id + ": " + errnoText(rc) +
              " (another process may hold it)";
        return false;
    }
    acquired = true;

    // VideoRecording asks for a steady-rate stream. On the Pi the role only
    // seeds defaults (YUV420, four buffers) that are overridden below, so
    // Viewfinder would serve as well.
    configuration = camera->generateConfiguration({libcamera::StreamRole::VideoRecording});
    if (configuration == nullptr || configuration->empty()) {
        err = "camera " + camera_id + " cannot provide a video stream";
        return false;
    }
    libcamera::StreamConfiguration& sc = configuration->at(0);
    const libcamera::Size requested(static_cast<unsigned int>(config.width_px),
                                    static_cast<unsigned int>(config.height_px));
    // The Pi ISP exposes no greyscale output through libcamera (an R8
    // request is reclassified as a raw sensor stream), so the Y plane of
    // YUV420 is the Y8 the runtime consumes.
    sc.pixelFormat = libcamera::formats::YUV420;
    sc.size        = requested;
    sc.bufferCount = kBufferCount;

    const libcamera::CameraConfiguration::Status status = configuration->validate();
    if (status == libcamera::CameraConfiguration::Invalid) {
        err = "camera " + camera_id + " rejects a " + requested.toString() +
              " YUV420 stream; pick a mode from rpicam-hello --list-cameras";
        return false;
    }
    if (sc.size != requested || sc.pixelFormat != libcamera::formats::YUV420) {
        // Calibration is tied to image geometry. Harmless adjustments to
        // padding/buffer count/colour space are permitted, size/format are not.
        err = "camera " + camera_id + " adjusted the requested " + requested.toString() +
              " YUV420 stream to " + sc.toString() + " (size " + sc.size.toString() +
              ", format " + sc.pixelFormat.toString() +
              "); the runtime never scales, pick a mode from rpicam-hello --list-cameras";
        return false;
    }

    rc = camera->configure(configuration.get());
    if (rc < 0) {
        err = "cannot configure camera " + camera_id + ": " + errnoText(rc);
        return false;
    }
    stream = sc.stream();
    stride = sc.stride;
    if (stream == nullptr || stride < requested.width ||
        requested.height > std::numeric_limits<std::size_t>::max() / stride) {
        err = "camera " + camera_id + " reports stride " + std::to_string(stride) +
              " for width " + std::to_string(requested.width);
        return false;
    }
    stream_note = sc.toString() + " stride " + std::to_string(stride) + " (Y plane as Y8)";

    allocator = std::make_unique<libcamera::FrameBufferAllocator>(camera);
    rc        = allocator->allocate(stream);
    if (rc < 0) {
        err = "cannot allocate frame buffers for camera " + camera_id + ": " + errnoText(rc);
        return false;
    }
    const std::vector<std::unique_ptr<libcamera::FrameBuffer>>& buffers =
        allocator->buffers(stream);
    if (buffers.empty()) {
        err = "camera " + camera_id + " allocated no frame buffers";
        return false;
    }
    const std::size_t y_bytes = static_cast<std::size_t>(stride) * requested.height;
    mapped.resize(buffers.size());
    for (std::size_t i = 0; i < buffers.size(); ++i) {
        libcamera::FrameBuffer* buffer = buffers[i].get();
        const auto&             planes = buffer->planes();
        if (planes.empty()) {
            err = "frame buffer " + std::to_string(i) + " has no planes";
            return false;
        }
        const libcamera::FrameBuffer::Plane& y_plane = planes[0];
        if (y_plane.offset == std::numeric_limits<unsigned int>::max() ||
            y_plane.length > std::numeric_limits<std::size_t>::max() -
                                 static_cast<std::size_t>(y_plane.offset)) {
            err = "frame buffer " + std::to_string(i) + " has an invalid Y-plane mapping";
            return false;
        }
        if (y_plane.length < y_bytes) {
            err = "frame buffer " + std::to_string(i) + " Y plane holds " +
                  std::to_string(y_plane.length) + " bytes, " + std::to_string(y_bytes) +
                  " needed";
            return false;
        }
        // every plane shares one dmabuf on the Pi; map from its start through
        // the Y plane once, for the life of the device
        const std::size_t length = static_cast<std::size_t>(y_plane.offset) + y_plane.length;
        void* base = mmap(nullptr, length, PROT_READ, MAP_SHARED, y_plane.fd.get(), 0);
        if (base == MAP_FAILED) {
            err = "cannot map frame buffer " + std::to_string(i) + ": " + errnoText(errno);
            return false;
        }
        mapped[i].base   = base;
        mapped[i].length = length;
        mapped[i].y      = static_cast<const uint8_t*>(base) + y_plane.offset;
        mapped[i].y_length = y_plane.length;
        mapped[i].fd = y_plane.fd.get();

        std::unique_ptr<libcamera::Request> request = camera->createRequest(i);
        if (request == nullptr) {
            err = "camera " + camera_id + " cannot create a capture request";
            return false;
        }
        rc = request->addBuffer(stream, buffer);
        if (rc < 0) {
            err = "cannot attach frame buffer " + std::to_string(i) + " to its request: " +
                  errnoText(rc);
            return false;
        }
        requests.push_back(std::move(request));
    }

    // fixed frame rate: both limits equal, microseconds. Exposure and gain
    // stay automatic; manual control is a documented follow-up.
    configured_frame_us = static_cast<int64_t>(std::llround(1000000.0 / config.frame_rate_hz));
    const int64_t limits[2]  = {configured_frame_us, configured_frame_us};
    const auto duration_control = camera->controls().find(&libcamera::controls::FrameDurationLimits);
    if (duration_control == camera->controls().end()) {
        err = "camera " + camera_id + " does not support FrameDurationLimits";
        return false;
    }
    const int64_t minimum_frame_us = duration_control->second.min().get<int64_t>();
    const int64_t maximum_frame_us = duration_control->second.max().get<int64_t>();
    if (configured_frame_us < minimum_frame_us || configured_frame_us > maximum_frame_us) {
        err = "camera " + camera_id + " cannot provide requested frame duration " +
              std::to_string(configured_frame_us) + " us; configured mode supports " +
              std::to_string(minimum_frame_us) + ".." + std::to_string(maximum_frame_us) + " us";
        return false;
    }
    start_controls.set(libcamera::controls::FrameDurationLimits,
                       libcamera::Span<const int64_t, 2>(limits));

    {
        std::lock_guard<std::mutex> lock(state_mutex);
        opened = true;
    }
    return startCapture(err);
}

bool LibcameraCamera::Impl::startCapture(std::string& err) {
    if (!connected) {
        camera->requestCompleted.connect(this, &Impl::onRequestCompleted);
        camera->disconnected.connect(this, &Impl::onDisconnected);
        connected = true;
    }
    {
        std::lock_guard<std::mutex> lock(queue_mutex);
        completed.clear();
        quit = false;
    }
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        started = false;
        failed = false;
        have_request_sequence = false;
        have_frame_sequence = false;
        last_error.clear();
    }
    // after a stop every request is idle (cancelled or never requeued);
    // reuse resets each one and keeps its buffer
    for (std::unique_ptr<libcamera::Request>& request : requests) {
        request->reuse(libcamera::Request::ReuseBuffers);
    }
    int rc = camera->start(&start_controls);
    if (rc < 0) {
        err = "cannot start camera " + camera_id + ": " + errnoText(rc);
        return false;
    }
    running = true;
    for (std::unique_ptr<libcamera::Request>& request : requests) {
        rc = camera->queueRequest(request.get());
        if (rc < 0) {
            err = "cannot queue a capture request on camera " + camera_id + ": " + errnoText(rc);
            camera->stop();
            running = false;
            return false;
        }
    }
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        started = true;
    }
    try {
        capture_thread = std::thread([this] { captureLoop(); });
    } catch (const std::exception& e) {
        err = std::string("cannot start the capture thread: ") + e.what();
        camera->stop();
        running = false;
        std::lock_guard<std::mutex> lock(state_mutex);
        started = false;
        return false;
    }
    return true;
}

void LibcameraCamera::Impl::stopCapture() {
    {
        std::lock_guard<std::mutex> lock(queue_mutex);
        quit = true;
    }
    queue_cv.notify_all();
    if (capture_thread.joinable()) {
        capture_thread.join();
    }
    if (running) {
        // in-flight requests complete as RequestCancelled through the slot
        // before stop() returns; nothing consumes them
        camera->stop();
        running = false;
    }
    std::lock_guard<std::mutex> lock(queue_mutex);
    completed.clear();
}

void LibcameraCamera::Impl::close() {
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        opened = false;
        started = false;
    }
    if (camera != nullptr) {
        stopCapture();
        if (connected) {
            camera->requestCompleted.disconnect(this);
            camera->disconnected.disconnect(this);
            connected = false;
        }
        requests.clear();
        for (MappedPlane& plane : mapped) {
            if (plane.base != nullptr) {
                munmap(plane.base, plane.length);
            }
        }
        mapped.clear();
        if (allocator != nullptr && stream != nullptr) {
            allocator->free(stream);
        }
        allocator.reset();
        stream = nullptr;
        configuration.reset();
        if (acquired) {
            camera->release();
            acquired = false;
        }
        camera.reset();
    }
    releaseManager(manager);
    std::lock_guard<std::mutex> lock(state_mutex);
    started = false;
}

void LibcameraCamera::Impl::fail(const std::string& reason) {
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        failed     = true;
        last_error = reason;
    }
    {
        std::lock_guard<std::mutex> lock(queue_mutex);
        quit = true;
    }
    queue_cv.notify_all();
}

void LibcameraCamera::Impl::onRequestCompleted(libcamera::Request* request) {
    // CameraManager thread: hand over and return; never block or call back
    // into libcamera here
    Completed done;
    done.request = request;
    done.at = HostClock::now();
    done.clock.host_before = done.at;
    timespec boot{};
    done.clock.valid = clock_gettime(CLOCK_BOOTTIME, &boot) == 0;
    done.clock.host_after = HostClock::now();
    if (done.clock.valid) {
        done.clock.source_now_ns = static_cast<int64_t>(boot.tv_sec) * 1000000000 + boot.tv_nsec;
    }
    {
        std::lock_guard<std::mutex> lock(queue_mutex);
        if (quit) { return; }
        completed.push_back(done);
    }
    queue_cv.notify_one();
}

void LibcameraCamera::Impl::onDisconnected() {
    fail("camera disconnected; reset the system after reconnecting the device");
}

void LibcameraCamera::Impl::captureLoop() {
    for (;;) {
        Completed done;
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            const auto timeout = std::chrono::milliseconds(
                std::max<int64_t>(1000, configured_frame_us / 1000 * 5));
            if (!queue_cv.wait_for(lock, timeout, [this] { return quit || !completed.empty(); })) {
                lock.unlock();
                fail("camera capture timed out waiting for a completed request; reset to retry");
                return;
            }
            if (quit) {
                return;
            }
            done = completed.front();
            completed.pop_front();
        }
        try {
            handleCompleted(done);
        } catch (const std::exception& e) {
            fail(std::string("capture processing failed: ") + e.what());
            return;
        }
    }
}

void LibcameraCamera::Impl::handleCompleted(const Completed& done) {
    libcamera::Request* request = done.request;
    if (request->status() != libcamera::Request::RequestComplete) {
        // Stop cancellation is ignored by the callback once quit is set.
        // Cancellation while running usually signals a hardware timeout.
        fail("camera cancelled an active capture request; reset to retry");
        return;
    }

    // read everything before reuse() clears the metadata
    const uint64_t          index  = request->cookie();
    libcamera::FrameBuffer* buffer = request->findBuffer(stream);
    bool usable = buffer != nullptr && index < mapped.size() &&
                        buffer->metadata().status == libcamera::FrameMetadata::FrameSuccess;
    const unsigned int frame_sequence = buffer != nullptr ? buffer->metadata().sequence : 0;
    const uint32_t     request_sequence = request->sequence();
    const std::optional<int64_t> sensor_ns =
        request->metadata().get(libcamera::controls::SensorTimestamp);
    const std::optional<int32_t> exposure_us =
        request->metadata().get(libcamera::controls::ExposureTime);
    const std::optional<int64_t> frame_duration_us =
        request->metadata().get(libcamera::controls::FrameDuration);

    std::shared_ptr<std::vector<uint8_t>> pixels;
    if (usable) {
        const std::size_t width  = static_cast<std::size_t>(config.width_px);
        const std::size_t height = static_cast<std::size_t>(config.height_px);
        const MappedPlane& plane = mapped[index];
        CpuRead access(plane.fd);
        if (!access.valid()) {
            fail("cannot begin CPU access to capture buffer: " + errnoText(errno));
            return;
        }
        pixels = std::make_shared<std::vector<uint8_t>>();
        const auto metadata_planes = buffer->metadata().planes();
        const std::size_t bytes_used = metadata_planes.empty() ? 0 :
            std::min<std::size_t>(metadata_planes[0].bytesused, plane.y_length);
        usable = copyY8Plane(plane.y, bytes_used, width, height, stride, *pixels);
        if (!access.finish()) {
            fail("cannot end CPU access to capture buffer: " + errnoText(errno));
            return;
        }
    }

    // the buffer goes back to the driver before the copy is published, so
    // the pool never waits on a consumer
    request->reuse(libcamera::Request::ReuseBuffers);
    const int rc = camera->queueRequest(request);
    if (rc < 0) {
        fail("cannot requeue a capture request: " + errnoText(rc));
    }

    std::lock_guard<std::mutex> lock(state_mutex);
    if (have_request_sequence && request_sequence != last_request_sequence + 1) {
        ++request_sequence_gaps;
    }
    have_request_sequence = true;
    last_request_sequence = request_sequence;
    if (!usable) {
        ++frame_errors;
        return;
    }
    if (have_frame_sequence && frame_sequence != last_frame_sequence + 1) {
        ++frame_sequence_gaps;
    }
    have_frame_sequence = true;
    last_frame_sequence = frame_sequence;

    CameraFrameData frame;
    frame.epoch      = epoch;
    if (sequence == std::numeric_limits<uint32_t>::max()) {
        frame.epoch = ++epoch;
        sequence = 0;
    }
    frame.sequence   = ++sequence;
    frame.receivedAt = done.at;
    frame.width_px   = static_cast<int>(config.width_px);
    frame.height_px  = static_cast<int>(config.height_px);
    frame.y8         = std::move(pixels);
    const auto stamp = sensor_ns.has_value() ? correlateCaptureTimestamp(*sensor_ns, done.clock)
                                            : std::nullopt;
    const auto exposure = estimateCaptureExposure(stamp, exposure_us,
        std::max(configured_frame_us, frame_duration_us.value_or(0)), done.at);
    frame.exposureAt = exposure.at;
    frame.exposure_uncertainty_ms = exposure.uncertainty_ms;
    frame.exposure_time_reliable = exposure.reliable;
    if (newest.has_value() && !newest_taken) {
        ++frames_replaced;
    }
    newest       = std::move(frame);
    newest_taken = false;
    ++frames_captured;
}

// ---- CameraDevice ----------------------------------------------------------

LibcameraCamera::LibcameraCamera(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

LibcameraCamera::~LibcameraCamera() = default;   // Impl closes in order

std::shared_ptr<CameraDevice> LibcameraCamera::open(const CameraCaptureConfig& config,
                                                    std::string&               err) {
    auto impl = std::make_unique<Impl>(config);
    if (!impl->open(err)) {
        return nullptr;   // ~Impl unwinds whatever was set up
    }
    return std::shared_ptr<CameraDevice>(new LibcameraCamera(std::move(impl)));
}

std::shared_ptr<CameraDevice> LibcameraCamera::dead(const CameraCaptureConfig& config,
                                                    const std::string&         reason) {
    auto impl        = std::make_unique<Impl>(config);
    impl->failed     = true;
    impl->last_error = reason;
    return std::shared_ptr<CameraDevice>(new LibcameraCamera(std::move(impl)));
}

bool LibcameraCamera::alive() const {
    std::lock_guard<std::mutex> lock(impl_->state_mutex);
    return impl_->started && !impl_->failed;
}

std::string LibcameraCamera::diagnostic() const {
    std::lock_guard<std::mutex> lock(impl_->state_mutex);
    const Impl&                 d = *impl_;
    if (!d.opened) {
        return d.last_error;
    }
    std::string out = d.version_note + "; camera " + d.camera_id + "; stream " + d.stream_note +
                      "; frames captured " + std::to_string(d.frames_captured) + ", replaced " +
                      std::to_string(d.frames_replaced) + ", errors " +
                      std::to_string(d.frame_errors) + "; request sequence gaps " +
                      std::to_string(d.request_sequence_gaps) + ", frame sequence gaps " +
                      std::to_string(d.frame_sequence_gaps);
    if (!d.last_error.empty()) {
        out += "; error: " + d.last_error;
    }
    return out;
}

const CameraIntrinsics* LibcameraCamera::intrinsics() const {
    return impl_->config.calibrated ? &impl_->config.intrinsics : nullptr;
}

FrameId LibcameraCamera::engineeringFrame() const { return impl_->config.engineering_frame; }

std::optional<CameraFrameData> LibcameraCamera::latestFrame(uint64_t after_epoch,
                                                            uint32_t after_sequence) {
    std::lock_guard<std::mutex> lock(impl_->state_mutex);
    Impl&                       d = *impl_;
    if (!d.newest.has_value()) {
        return std::nullopt;
    }
    const bool later = d.newest->epoch > after_epoch ||
                       (d.newest->epoch == after_epoch && d.newest->sequence > after_sequence);
    if (!later) {
        return std::nullopt;
    }
    d.newest_taken = true;
    return d.newest;   // the pixels are shared, not copied
}

void LibcameraCamera::reset() {
    Impl& d = *impl_;
    // Reacquire rather than merely restarting old buffers: this also
    // recovers a failed initial open or a device reconnected since failure.
    d.close();
    {
        std::lock_guard<std::mutex> lock(d.state_mutex);
        d.epoch += 1;
        d.sequence     = 0;
        d.newest_taken = true;
        d.newest.reset();
        d.started = false;
    }
    std::string err;
    if (!d.open(err)) {
        d.close();
        d.fail("restart failed: " + err);
    }
}

} // namespace navigatr
