# Reference template — the canonical formula lives in mavlink/homebrew-tap
# and is automatically updated by .github/workflows/release.yml on each release.
class MavsimViewer < Formula
  desc "3D MAVLink vehicle viewer for SITL simulation"
  homepage "https://github.com/mavlink/mavsim-viewer"
  url "https://github.com/mavlink/mavsim-viewer/releases/download/vVERSION/mavsim-viewer-VERSION.tar.gz"
  sha256 "TODO"
  license "BSD-3-Clause"

  depends_on "cmake" => :build

  def install
    system "cmake", "-S", ".", "-B", "build",
           "-DCMAKE_BUILD_TYPE=Release",
           *std_cmake_args
    system "cmake", "--build", "build"
    system "cmake", "--install", "build", "--prefix", prefix
  end

  test do
    assert_predicate bin/"mavsim-viewer", :executable?
  end
end
