#ifndef RT_RUNNER_H
#define RT_RUNNER_H

class RTRunner
{
private:
    bool infererOrder();

public:
    RTRunner();
    virtual ~RTRunner();

    void configure();
    void shutdown();

    bool loadOrocosComponent();
    bool unloadOrocosComponent();

    void setFrequency();
    void activateRTLoopCallback();
    void deactivateRTLoopCallback();
};

#endif /* RT_RUNNER_H */
