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

#include <alproxies/almemoryproxy.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alfacedetectionproxy.h>
#include <alvalue/alvalue.h>

#include <althread/almutex.h>

struct Human {
    std::string id;
    float x,y,z;
};

class InteractionMonitor
{
public:

    InteractionMonitor();

    virtual ~InteractionMonitor();

    void init(boost::shared_ptr<AL::ALBroker> broker);
    bool connectProxies(boost::shared_ptr<AL::ALBroker> broker);

    void getHumans(std::vector<Human>& detectedHumans);

private:
    void processFaces(const AL::ALValue &faces, std::vector<Human>& humans);
    Human makeHuman(const std::string& name, float yaw, float pitch);
    void onWord(const std::string &key, const AL::ALValue &value, const AL::ALValue &msg);
    void onSound(const std::string &key, const AL::ALValue &value, const AL::ALValue &msg);

    boost::shared_ptr<AL::ALMotionProxy> m_motionProxy;
    boost::shared_ptr<AL::ALMemoryProxy> m_memoryProxy;
    boost::shared_ptr<AL::ALFaceDetectionProxy> m_faceProxy;
    AL::ALValue m_dataNamesList;

    boost::shared_ptr<AL::ALMutex> fSoundCallbackMutex;
    boost::shared_ptr<AL::ALMutex> fWordCallbackMutex;

    unsigned int facesCount;

    bool isSeparate(float yaw, float pitch);

    std::map<int, std::pair<float, float> > m_lastSeenHumans;
};

#endif  // EVENTS_EVENTS_H
