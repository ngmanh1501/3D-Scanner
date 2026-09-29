#include "path_utils.h"
#include <filesystem>
#include <algorithm>

using namespace std;
namespace fs = std::filesystem;

int extractNumber(const string& s)
{
    size_t pos = s.find_last_of('_');
    if (pos == string::npos)
        return -1;

    return stoi(s.substr(pos + 1));
}

vector<string> listDirectories(const string& path)
{
    vector<string> dirs;
    if (!fs::exists(path)) return dirs;
    for (const auto& entry : fs::directory_iterator(path))
        if (entry.is_directory())
            dirs.push_back(entry.path().filename().string());
    sort(dirs.begin(), dirs.end(),
    [](const string& a, const string& b)
    {
        return extractNumber(a) < extractNumber(b);
    });
    return dirs;
}

vector<string> listFiles(const string& path, const string& ext)
{
    vector<string> files;
    if (!fs::exists(path)) return files;
    for (const auto& entry : fs::directory_iterator(path))
        if (entry.is_regular_file() && entry.path().extension() == ext)
            files.push_back(entry.path().filename().string());
    sort(files.begin(), files.end());
    return files;
}
