#ifndef __FPS_COUNTER_H_
#define __FPS_COUNTER_H_

class FPSCounter
{
public:
    FPSCounter(const char* name = "");

    void name(const char* name) { m_name = name; }

    void reset();

    void update();

protected:
    const char* m_name;
    uint32_t m_last_time;
    uint32_t m_num_frames = 0;
};

#endif // __FPS_COUNTER_H_
