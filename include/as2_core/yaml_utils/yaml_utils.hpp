#ifndef __AS2_YAML_UTILS_HPP__
#define __AS2_YAML_UTILS_HPP__

#include <yaml-cpp/yaml.h>

#include <bitset>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace as2 {

std::filesystem::path get_project_export_path_from_xml_path(const std::filesystem::path& xml_path);
std::vector<std::filesystem::path> find_yaml_files_inside(const std::filesystem::path& dir);

YAML::Node find_tag_across_multiple_yaml_files(const std::vector<std::filesystem::path>& yaml_files,
                                               const std::string& tag);

uint8_t parse_uint_from_string(const std::string& str);

// Recursive function to find the tag inside the yaml file
YAML::Node find_tag_in_yaml_node(const YAML::Node& node, const std::string& tag);

template <typename T = std::string>
std::vector<T> find_tag_in_yaml_file(const std::filesystem::path& yaml_file,
                                     const std::string& tag) {
  std::vector<T> result;
  std::ifstream config_file(yaml_file);
  // check if file exists
  if (!config_file.good()) {
    throw std::runtime_error("Could not open yaml file: " + yaml_file.string());
  }
  YAML::Node node = YAML::LoadFile(yaml_file);
  for (const auto& elem : find_tag_in_yaml_node(node, tag)) {
    T value = elem.as<T>();
    result.emplace_back(value);
  }
  return result;
};

template <typename T = std::string>
std::vector<T> find_tag_from_project_exports_path(const std::filesystem::path& project_exports_path,
                                                  const std::string& tag) {
  std::vector<T> values;
  std::vector<std::filesystem::path> yaml_files = find_yaml_files_inside(project_exports_path);
  if (yaml_files.size() == 0) {
    std::cerr << "Could not find yaml files with available modes" << std::endl;
    return std::vector<T>();
  }

  for (const auto& yaml_file : yaml_files) {
    auto tags = find_tag_in_yaml_file<T>(yaml_file, tag);
    if (tags.size() > 0) {
      values = tags;
    }
  }
  return values;
}

}  // namespace as2
#endif  // __AS2_YAML_UTILS_HPP__