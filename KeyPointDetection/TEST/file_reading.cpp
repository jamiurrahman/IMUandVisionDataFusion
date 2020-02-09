#include <string>
#include <iostream>
#include <filesystem>
namespace fs = std::filesystem;

int main() {
    std::string path = "../data/*.png";
    for (const auto & entry : fs::directory_iterator(path))
        std::cout << entry.path() << std::endl;
}
