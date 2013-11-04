#define FACE_DETECTION_RATE 200 //ms
#define AVERAGEHUMANDISTANCE 1.0 // distance in meter between human and robot

#include <ros/ros.h> // for logging only

#include "interactions.h"

#include <alvalue/alvalue.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <althread/alcriticalsection.h>

#include <cmath>       /* abs sin cos */
#define PI 3.14159265
#define FACE_SEPARATION 0.05 // value, in rad, that must separate two faces to consider them distinct

#define NB_VIEWS_BEFORE_LEARNING 5

using namespace std;

// Helper to generate random ID.
// taken from http://stackoverflow.com/questions/440133
void gen_random(char *s, const int len) {
    static const char alphanum[] =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz";

    for (int i = 0; i < len; ++i) {
        s[i] = alphanum[rand() % (sizeof(alphanum) - 1)];
    }

    s[len] = 0;
}


InteractionMonitor::InteractionMonitor():
      facesCount(0),
      fSoundCallbackMutex(AL::ALMutex::createALMutex()),
      fWordCallbackMutex(AL::ALMutex::createALMutex()),
      humanBuffer(NB_VIEWS_BEFORE_LEARNING)
{

    m_dataNamesList = AL::ALValue::array("FaceDetected");

}

InteractionMonitor::~InteractionMonitor() {
    m_faceProxy->unsubscribe("nao_human_face_detection");
}

void InteractionMonitor::init(boost::shared_ptr<AL::ALBroker> broker) {

    connectProxies(broker);

    m_faceProxy->clearDatabase();
    m_faceProxy->subscribe("nao_human_face_detection", FACE_DETECTION_RATE, 0.0);
}

void InteractionMonitor::getHumans(map<string, Human>& humans) {
    AL::ALValue faces = m_memoryProxy->getData("FaceDetected");
    processFaces(faces, humans);
}

Human InteractionMonitor::makeHuman(const std::string& name, float yaw, float pitch) {

    // using physics convention from http://en.wikipedia.org/wiki/Spherical_coordinate_system
    Human h;

    h.id = name;
    time(&h.lastseen);

    float theta = PI/2 - pitch;
    float phi = yaw;

    h.x = AVERAGEHUMANDISTANCE * sin(theta) * cos(phi);
    h.y = AVERAGEHUMANDISTANCE * sin(theta) * sin(phi);
    h.z = -AVERAGEHUMANDISTANCE * cos(theta);
    
    ROS_INFO("%s: x:%.2f, y:%.2f, z:%.2f", name.c_str(), h.x, h.y, h.z);
    
    return h;
}


void InteractionMonitor::processFaces(const AL::ALValue &faces, map<string, Human>& humans) {

    try {
        /** Check that there are faces effectively detected. */
        if (faces.getSize() < 2 ) {
            if (facesCount != 0) {
                ROS_INFO("No face detected");
                humanBuffer = NB_VIEWS_BEFORE_LEARNING;
                facesCount = 0;
            }
            return;
        }
        /** Check the number of faces from the FaceInfo field, and check that it has
        * changed from the last event.*/
        if (faces[1].getSize() - 1 != facesCount) {
            ROS_INFO("%d face(s) detected.", faces[1].getSize() - 1);
            /** Update the current number of detected faces. */
            facesCount = faces[1].getSize() - 1;
        }
    }
    catch (const AL::ALError& e) {
        ROS_ERROR(e.what());
    }

    for (int i = 0; i < facesCount; ++i) {
       
        int naoqiFaceId = faces[1][i][1][0];
        float confidence = faces[1][i][1][1];

        // Face > [FaceInfo] > FaceInfo > ShapeInfo > yaw
        float yaw = faces[1][i][0][1];
        // Face > [FaceInfo] > FaceInfo > ShapeInfo > pitch
        float pitch = faces[1][i][0][2];

        ROS_INFO("Face (id: %d) confidence:%.2f yaw: %.2f, pitch: %.2f", naoqiFaceId, confidence, yaw, pitch);
        //ROS_INFO("Face (id: %d) reco confidence: %.2f", naoqiFaceId, confidence);
        //ROS_INFO("Time_Filtered_Reco_Info: %s", faces[1][i].toString().c_str());

        int candidateId = facesCloseTo(yaw, pitch);

        if (naoqiFaceId == -1 && candidateId == -1)  // seen unknown new face
        {
            if (humanBuffer > 0) {
                humanBuffer--;
            }
            else {
                humanBuffer = NB_VIEWS_BEFORE_LEARNING;
                char faceId[6];
                gen_random(faceId, 5);
                ROS_INFO("Unknown face (confidence: %.2f). Learning it as <%s>", confidence, faceId);
                if(!m_faceProxy->learnFace(faceId))
                {
                    ROS_ERROR("Could not learn the new face!");
                }
            }
        }
        else { // face recognized!

            humanBuffer = NB_VIEWS_BEFORE_LEARNING;
            // too close to an existing face? re-use the existing face!
            if (candidateId != -1) {
                naoqiFaceId = candidateId;
            }
            else {
                assert(naoqiFaceId != -1);
                string label = faces[1][i][1][2];
                id2label[naoqiFaceId] = label;
            }

            string label = id2label[naoqiFaceId];
            // save the yaw,pitch pose of the recognized face, to
            // prevent jitter at next round of detections
            m_lastSeenHumans[naoqiFaceId] = make_pair(yaw, pitch);


            humans[label] = makeHuman(label, yaw, pitch);
        }
    }

}

/** This method check if two faces are 'far enough' to be likely to be 2 different
persons. This prevent over-learning of badly detected faces.
**/
int InteractionMonitor::facesCloseTo(float yaw, float pitch) {
    map<int, pair<float, float> >::iterator it;

    for (it=m_lastSeenHumans.begin(); it!=m_lastSeenHumans.end(); ++it)
    {
        if (abs(yaw - it->second.first) < FACE_SEPARATION
            && abs(pitch - it->second.second) < FACE_SEPARATION)
        {
            return it->first;
        }
    }
    return -1;
}

void InteractionMonitor::onWord(const std::string &key, const AL::ALValue &value, const AL::ALValue &msg) {
    AL::ALCriticalSection section(fWordCallbackMutex);
    //addHumanDetected("word", 0.0, 0.0);

}

void InteractionMonitor::onSound(const std::string &key, const AL::ALValue &value, const AL::ALValue &msg) {
    AL::ALCriticalSection section(fSoundCallbackMutex);
    //addHumanDetected("sound", 0.0, 0.0);

}



bool InteractionMonitor::connectProxies(boost::shared_ptr<AL::ALBroker> broker) {

    // connect to the NAOqi proxies
    if (!broker)
    {
        ROS_ERROR("Broker is not ready. Have you called connectNaoQi()?");
        return false;
    }
    try
    {
        m_motionProxy = boost::shared_ptr<AL::ALMotionProxy>(new AL::ALMotionProxy(broker));
    }
    catch (const AL::ALError& e)
    {
        ROS_ERROR("Could not create ALMotionProxy.");
        return false;
    }
    try
    {
        m_memoryProxy = boost::shared_ptr<AL::ALMemoryProxy>(new AL::ALMemoryProxy(broker));
    }
    catch (const AL::ALError& e)
    {
        ROS_ERROR("Could not create ALMemoryProxy.");
        return false;
    }
    try
    {
        m_faceProxy = boost::shared_ptr<AL::ALFaceDetectionProxy>(new AL::ALFaceDetectionProxy(broker));
    }
    catch (const AL::ALError& e)
    {
        ROS_ERROR("Could not create ALFaceDetectionProxy.");
        return false;
    }
    ROS_INFO("All NAOqi proxies are ready.");
    return true;

}

