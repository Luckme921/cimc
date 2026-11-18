#pragma once
#include <weld_seam_sdk/weld_seam_sdk.hpp>

namespace gooseneck_sdk {
// Same transport/result contract, independent implementation and shared library.
using RunOptions = weld_seam_sdk::RunOptions;
using RunResult = weld_seam_sdk::RunResult;
RunResult run(const RunOptions& options);
const char* version();
}
