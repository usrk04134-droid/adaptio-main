#pragma once

#include <yaml-cpp/yaml.h>

#include <array>
#include <optional>
#include <stdexcept>
#include <string>

namespace test_utils {

struct ExpectedABWPoints {
  std::array<double, 7> xs{};
  std::array<double, 7> ys{};
};

inline auto ExtractSevenDoubles(const YAML::Node& node) -> std::optional<std::array<double, 7>> {
  if (!node || !node.IsSequence() || node.size() != 7) {
    return std::nullopt;
  }

  std::array<double, 7> out{};
  for (std::size_t i = 0; i < 7; ++i) {
    out[i] = node[i].as<double>();
  }
  return out;
}

inline auto ParseExpectedFromNode(const YAML::Node& item) -> std::optional<ExpectedABWPoints> {
  // Support either
  // expected_abw: { xs: [...], ys: [...] }
  // or
  // expected_xs: [...], expected_ys: [...]

  if (const auto expected = item["expected_abw"]; expected && expected.IsMap()) {
    auto maybe_xs = ExtractSevenDoubles(expected["xs"]);
    auto maybe_ys = ExtractSevenDoubles(expected["ys"]);
    if (maybe_xs && maybe_ys) {
      return ExpectedABWPoints{*maybe_xs, *maybe_ys};
    }
  }

  auto maybe_xs = ExtractSevenDoubles(item["expected_xs"]);
  auto maybe_ys = ExtractSevenDoubles(item["expected_ys"]);
  if (maybe_xs && maybe_ys) {
    return ExpectedABWPoints{*maybe_xs, *maybe_ys};
  }

  return std::nullopt;
}

inline auto LoadExpectedABWFromYamlById(const std::string& yaml_path, int image_id)
    -> std::optional<ExpectedABWPoints> {
  const YAML::Node root = YAML::LoadFile(yaml_path);

  // Support two shapes:
  // 1) Top-level key: data_set: [ { id: 3, image: ..., expected_abw: { xs, ys } }, ... ]
  // 2) Top-level is a sequence of entries with id/image/expected_abw

  if (const auto data_set = root["data_set"]; data_set && data_set.IsSequence()) {
    for (const auto& item : data_set) {
      if (item["id"] && item["id"].as<int>() == image_id) {
        return ParseExpectedFromNode(item);
      }
    }
    return std::nullopt;
  }

  if (root.IsSequence()) {
    for (const auto& item : root) {
      if (item["id"] && item["id"].as<int>() == image_id) {
        return ParseExpectedFromNode(item);
      }
    }
    return std::nullopt;
  }

  // Also support a map keyed by id as string
  if (root.IsMap()) {
    const std::string id_key = std::to_string(image_id);
    if (root[id_key]) {
      return ParseExpectedFromNode(root[id_key]);
    }
  }

  return std::nullopt;
}

inline auto LoadExpectedABWFromYamlByImage(const std::string& yaml_path, const std::string& image_name)
    -> std::optional<ExpectedABWPoints> {
  const YAML::Node root = YAML::LoadFile(yaml_path);

  if (const auto data_set = root["data_set"]; data_set && data_set.IsSequence()) {
    for (const auto& item : data_set) {
      if (item["image"] && item["image"].as<std::string>() == image_name) {
        return ParseExpectedFromNode(item);
      }
    }
    return std::nullopt;
  }

  if (root.IsSequence()) {
    for (const auto& item : root) {
      if (item["image"] && item["image"].as<std::string>() == image_name) {
        return ParseExpectedFromNode(item);
      }
    }
    return std::nullopt;
  }

  // Also support a map keyed by image name
  if (root.IsMap() && root[image_name]) {
    return ParseExpectedFromNode(root[image_name]);
  }

  return std::nullopt;
}

}  // namespace test_utils

