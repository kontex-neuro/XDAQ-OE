from conan import ConanFile
from conan.tools.cmake import CMakeToolchain


class libxdaq(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeDeps", "VirtualRunEnv"

    def requirements(self):
        self.requires("boost/1.85.0")
        self.requires("zlib/1.3.1")
        self.requires("fmt/10.2.1")
        self.requires("nlohmann_json/3.10.5")
        self.requires("spdlog/1.13.0")

    def configure(self):
        self.options["json-schema-validator"].shared = False

    def generate(self):
        tc = CMakeToolchain(self)
        tc.generate()
