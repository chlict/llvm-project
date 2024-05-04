set(LLVM_LINK_COMPONENTS
  Support
  )

add_clang_executable(edsl-parser EDSLParser.cpp)

target_link_libraries(edsl-parser
  PRIVATE
  clangAST
  clangBasic
  clangFrontend
  clangSerialization
  clangTooling
  )
