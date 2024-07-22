from conan import ConanFile
from conan.tools.cmake import CMakeToolchain


class libxdaq(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeDeps", "VirtualRunEnv"

    def requirements(self):
        self.requires("boost/1.81.0")
        self.requires("zlib/1.3.1")
        self.requires("fmt/10.2.1")
        self.requires("nlohmann_json/3.11.3")
        self.requires("spdlog/1.13.0")

    def configure(self):
        self.options["boost/*"].with_stacktrace_backtrace = False
        self.options["boost/*"].without_locale = True
        self.options["boost/*"].without_stacktrace = True

    def generate(self):
        tc = CMakeToolchain(self)
        tc.generate()
