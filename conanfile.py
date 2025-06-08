from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, cmake_layout


class libxdaq(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeDeps", "VirtualRunEnv"

    def requirements(self):
        self.requires("fmt/10.2.1")
        self.requires("nlohmann_json/3.11.3")
        self.requires("spdlog/1.13.0")
        self.requires("libxdaq/0.3.0")

    def generate(self):
        tc = CMakeToolchain(self)
        tc.generate()
    
    def layout(self):
        cmake_layout(self)
