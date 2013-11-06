/**
 * @author
 *
 * This file was generated by Aldebaran Robotics ModuleGenerator
 */

#ifndef EVENTS_EVENTS_H
#define EVENTS_EVENTS_H

#include <boost/shared_ptr.hpp>
#include <string>
#include <vector>
#include <map>
#include <utility> //std::pair, std::make_pair
#include <time.h>

#include <alproxies/almemoryproxy.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alfacedetectionproxy.h>
#include <alproxies/alaudiosourcelocalizationproxy.h>
#include <alvalue/alvalue.h>

#include <althread/almutex.h>

struct Human {
    std::string id;
    float x,y,z;
    time_t lastseen;
    bool speaking;
};

class InteractionMonitor
{
public:

    InteractionMonitor();

    virtual ~InteractionMonitor();

    void init(boost::shared_ptr<AL::ALBroker> broker);
    bool connectProxies(boost::shared_ptr<AL::ALBroker> broker);

    void getHumans(std::map<std::string, Human>& detectedHumans);

private:
    void processFaces(const AL::ALValue &faces, std::map<std::string, Human>& humans);
    void identifySpeaker(const AL::ALValue &sounds, std::map<std::string, Human>& humans);
    Human makeHuman(const std::string& name, float yaw, float pitch);
    float distance(const Human& h1, const Human& h2);

    boost::shared_ptr<AL::ALMotionProxy> m_motionProxy;
    boost::shared_ptr<AL::ALMemoryProxy> m_memoryProxy;
    boost::shared_ptr<AL::ALFaceDetectionProxy> m_faceProxy;
    boost::shared_ptr<AL::ALAudioSourceLocalizationProxy> m_soundSourceProxy;
    AL::ALValue m_dataNamesList;

    boost::shared_ptr<AL::ALMutex> fSoundCallbackMutex;
    boost::shared_ptr<AL::ALMutex> fWordCallbackMutex;

    unsigned int facesCount;

    int facesCloseTo(float yaw, float pitch);

    std::map<int, std::pair<float, float> > m_lastSeenHumans;
    std::map<int, std::string> id2label;
    int humanBuffer;
};

#endif  // EVENTS_EVENTS_H

