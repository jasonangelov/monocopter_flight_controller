#ifndef FLIGHT_CONTROL_CONFIG_H
#define FLIGHT_CONTROL_CONFIG_H

#include <cstdint>

namespace monocopter {

inline constexpr float kDegToRad = 0.01745329252f;
inline constexpr std::uint32_t kAltitudeValidMs = 300;

inline constexpr int kServoNeutralUs = 1500;
inline constexpr int kServoRangeUs = 250;
inline constexpr int kEscOffUs = 1000;
inline constexpr int kEscMinUs = 1000;
inline constexpr int kEscMaxUs = 2000;
inline constexpr int kEscHoverUs = 1200;

inline constexpr int kFin1OffsetUs = -100;
inline constexpr int kFin2OffsetUs = -100;
inline constexpr int kFin3OffsetUs = 75;
inline constexpr int kFin4OffsetUs = 75;

}  // namespace monocopter

#endif
