#include "fps_counter.hpp"
    
FPSCounter::FPSCounter(const std::string& name) : m_name(name)
{
    reset();
}

void FPSCounter::reset()
{
    m_last_time = micros();
    m_num_frames = 0;
}

void FPSCounter::update()
{
    m_num_frames++;
    uint32_t curr_time = micros();
    // INFO("read_frames: curr_time=%ld, last_time=%ld, diff=%ld", curr_time, last_time, curr_time - last_time);
    if ((curr_time - m_last_time) >= MICROS_PER_SEC)
    {
        float val = ((float)m_num_frames) * MICROS_PER_SEC / (curr_time - m_last_time);
        Serial.print(m_name);
        Serial.print(": fps=");
        Serial.println(val);

        m_last_time = curr_time;    
        m_num_frames = 0;
    }
}

