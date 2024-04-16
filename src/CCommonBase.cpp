/**
 * CCommonBase.cpp - inheritable class with common functions
 * 2024-02-08
 * vika <https://github.com/hi-im-vika>
 */

#include "../include/CCommonBase.hpp"

CCommonBase::~CCommonBase() = default;

void CCommonBase::run() {
    std::thread thread_for_updating(update_thread, this);
    thread_for_updating.detach();
    do {
        // we started at this time
        _perf_draw_start = std::chrono::steady_clock::now();
        // do draw
        draw();
        // update elapsed time since draw
        int since_start = (int) std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - _perf_draw_start).count();
        // announce elapsed time from draw alone
        spdlog::debug("Completed in " + std::to_string(since_start) + " ms");
        // after blocking complete, update total elapsed time
        _perf_draw = since_start;
    } while (!_do_exit);
    if (thread_for_updating.joinable()) thread_for_updating.join();
}

void CCommonBase::update_thread(CCommonBase *who_called_me) {
    while (!(who_called_me->_do_exit)) {
        who_called_me->_perf_update_start = std::chrono::steady_clock::now();
        who_called_me->update();
        who_called_me->_perf_update = (int) std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - who_called_me->_perf_update_start).count();
    }
}

void CCommonBase::draw_thread(CCommonBase *who_called_me) {
    while (!(who_called_me->_do_exit)) {
        who_called_me->_perf_draw_start = std::chrono::steady_clock::now();
        who_called_me->draw();
        who_called_me->_perf_draw = (int) std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - who_called_me->_perf_draw_start).count();
    }
}