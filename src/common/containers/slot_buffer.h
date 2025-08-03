#pragma once
#include <cmath>
#include <memory>
#include <optional>

namespace common::containers {

template <typename V>
class SlotBuffer {
 public:
  explicit SlotBuffer(size_t number_of_slots, double wrap_value)
      : data_(std::make_unique<std::optional<std::pair<double, V>>[]>(number_of_slots)),
        number_of_slots_(number_of_slots),
        wrap_value_(wrap_value),
        slot_size_(wrap_value_ / number_of_slots) {}
  ~SlotBuffer() = default;

  SlotBuffer(const SlotBuffer &other) = delete;
  SlotBuffer(SlotBuffer &other)       = delete;

  auto operator=(const SlotBuffer<V> &other) -> SlotBuffer<V> & {
    if (this != other) {
      data_            = other.data_;
      number_of_slots_ = other.number_of_slots_;
      wrap_value_      = other.wrap_value_;
      slot_size_       = other.slot_size_;
    }
  }

  auto operator=(SlotBuffer &&) -> SlotBuffer & = delete;

  auto Store(double index, V value) {
    filled_slots_               += !data_[CalculateSlot(index)].has_value() ? 1 : 0;
    data_[CalculateSlot(index)]  = {index, value};
  }

  auto Get(double index) -> std::optional<std::pair<double, V>> { return data_[CalculateSlot(index)]; }

  auto Clear() { data_ = std::make_unique<std::optional<std::pair<double, V>>[]>(number_of_slots_); }

  auto Empty() -> bool {
    for (int i = 0; i < number_of_slots_; i++) {
      if (data_[i].has_value()) {
        return false;
      }
    }

    return true;
  }

  auto Filled() -> bool { return filled_slots_ == number_of_slots_; }
  auto FilledSlots() -> size_t { return filled_slots_; }
  auto Slots() -> size_t { return number_of_slots_; }

 protected:
  std::unique_ptr<std::optional<std::pair<double, V>>[]> data_;
  size_t number_of_slots_{};
  double wrap_value_{};
  double slot_size_{};
  size_t filled_slots_{};

  auto CalculateSlot(double index) const -> size_t {
    return static_cast<size_t>(std::fmod(index, wrap_value_) / slot_size_);
  }
};

}  // namespace common::containers
