#pragma once

#include <ptm/portem.hpp>
#include <glm/glm.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <chrono>
#include <phtree/phtree.h>

namespace kin {
    using namespace improbable::phtree;

    inline auto start = std::chrono::high_resolution_clock::now();

    inline std::chrono::high_resolution_clock::time_point now_tp() {
        return std::chrono::high_resolution_clock::now();
    }

    template<typename measure>
    typename measure::rep now() {
        using namespace std::chrono;

        auto end = now_tp(); 
        measure dur = duration_cast<measure>(end - start);

        return dur.count();
    }

    template<typename rep, typename period>
    rep now() {
        using namespace std::chrono;
        using durationRP = duration<rep, period>;

        auto end = now_tp(); 
        durationRP dur = duration_cast<durationRP>(end - start);

        return dur.count();
    }
    void print_test();
    class profiler_t {
    public:
        int create_profile(const char* name);
        void start_profile(int id);
        void end_profile(int id);
        void print_profiles(int denom);

    private:
        struct profile_t {
            profile_t(const char* name)
                : name(name), dur(0), amount_called(0) {}

            const char* name;
            std::chrono::high_resolution_clock::time_point start;
            int64_t amount_called;
            int64_t dur;            
        };

        std::vector<profile_t> profilers;
    };

    constexpr float float_infinity = std::numeric_limits<float>::infinity();
    constexpr float float_max = std::numeric_limits<float>::max();
    constexpr float float_min = std::numeric_limits<float>::min();

    inline float cross(glm::vec2 a, glm::vec2 b) {
        return a.x * b.y - a.y * b.x;
    }

    inline bool nearly_equal(float a, float b, float max = 0.0001f) {
        return glm::abs(a - b) < max;
    }

    inline bool nearly_equal(glm::vec2 a, glm::vec2 b, float max = 0.0001f) {
        return glm::abs(a.x - b.x) < max && glm::abs(a.y - b.y) < max;
    }

    inline float sqaure(float value) {
        return value * value;
    }
}