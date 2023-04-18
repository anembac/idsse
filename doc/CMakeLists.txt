#
# Setup API documentation based on doxygen
#
find_package(
  Doxygen
  OPTIONAL_COMPONENTS dot mscgen dia
)

set(DOXYGEN_PROJECT_NAME "idsse")
set(DOXYGEN_PROJECT_BRIEF "intrusion detection system sim env")

# some additional doxygen settings for macro handling
set(DOXYGEN_ENABLE_PREPROCESSING "YES")
set(DOXYGEN_MACRO_EXPANSION "YES")
set(DOXYGEN_EXPAND_ONLY_PREDEF "YES")

# set expansion for enum related macros to empty to avoid errors
# see: https://marc.info/?l=doxygen-users&m=117260901714428
set(DOXYGEN_PREDEFINED
    "EZC2X_REFLECT_ENUM(x, y))="
    "EZC2X_ENUM_STREAM_OUTPUT(x)="
    "EZC2X_ENUM_STREAM_INPUT(x)="
    "EZC2X_ENUM_STREAM_INPUT_EXACT(x)"
)

if(CHECK_DOC_STRICT)
  set(DOXYGEN_WARN_AS_ERROR "YES")
  set(DOXYGEN_WARN_NO_PARAMDOC "YES")
endif()

doxygen_add_docs(
  doc_api
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/doc
  COMMENT "Generate API doc using doxygen"
)
