# copy_assets.cmake: copies the viewer (web/) and field definitions (fields/*.json) next to the
# receiver. Runs on every build; a missing source folder is skipped with a note.
#   cmake -DWEB_SRC=... -DFIELDS_SRC=... -DDEST=... -P copy_assets.cmake

if(IS_DIRECTORY "${WEB_SRC}")
    file(COPY "${WEB_SRC}/" DESTINATION "${DEST}/web" PATTERN "tests" EXCLUDE)
else()
    message(STATUS "spectagatr: no ${WEB_SRC} yet, viewer files not copied")
endif()

if(IS_DIRECTORY "${FIELDS_SRC}")
    file(COPY "${FIELDS_SRC}/" DESTINATION "${DEST}/fields" FILES_MATCHING PATTERN "*.json")
else()
    message(STATUS "spectagatr: no ${FIELDS_SRC} yet, fields not copied")
endif()
