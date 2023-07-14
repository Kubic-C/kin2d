#include "base.hpp"

using namespace std::chrono;

namespace kin {
    void print_test() {
        printf("Kin linkage test\n");
    }

    int  profiler_t::create_profile(const char* name) {
        int id =  profilers.size();

        profilers.emplace_back(name);

        return id;
    }

    void profiler_t::start_profile(int id) {
        profilers[id].start = now_tp();
    }

    void profiler_t::end_profile(int id) {
        profilers[id].dur += std::chrono::duration_cast<microseconds>(now_tp() - profilers[id].start).count(); 
        profilers[id].amount_called++;
    }

    void profiler_t::print_profiles(int denom) {
        for(auto& profile : profilers) {
            int called = profile.amount_called;
            if(denom != -1) {
                called = denom;
            }
            
            if(called == 0)
                continue; // dont want division by 0 error


            printf("#%s : %fus\n", profile.name, (float)profile.dur / (float)called);
            profile.dur = 0;
            profile.amount_called = 0;
        }
    }
}