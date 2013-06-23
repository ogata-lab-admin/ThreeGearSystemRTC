#include "HandTrackingMessage.h"

#include <cstdio>

#include <array>
#include <sstream>
#include <memory>
#include <cassert>
#include <cstdint>

#ifndef WIN32
#include <xlocale.h>
#endif

#ifdef WIN32
typedef _locale_t LocaleType;
#else
typedef locale_t LocaleType;
#endif

namespace HandTrackingClient
{

Vector3f
Vector3f::cross (const Vector3f& rhs) const
{
    const Vector3f& a = *this;
    const Vector3f& b = rhs;
    return Vector3f (a.y*b.z - a.z*b.y,
                     a.z*b.x - a.x*b.z,
                     a.x*b.y - a.y*b.x);
}

void
Vector3f::normalize()
{
    const float len = norm();
    if (len == 0.0f)
        return;

    x /= len;
    y /= len;
    z /= len;
}

std::ostream& 
operator<< (std::ostream& os, const Vector3f& rhs)
{
    os << rhs.x << " " << rhs.y << " " << rhs.z;
    return os;
}

Quaternionf
Quaternionf::conjugate() const
{
    return Quaternionf (-v, w);
}

void
Quaternionf::normalize()
{
    const float sqrLen = v.squaredNorm() + w*w;
    const float len = std::sqrt(sqrLen);
    if (len == 0.0f)
        return;

    v /= len;
    w /= len;
}

Quaternionf operator* (const Quaternionf& a, const Quaternionf& b)
{
    // From Wikipedia:
    // http://en.wikipedia.org/wiki/Quaternion#Scalar_and_vector_parts
    return Quaternionf (a.w*b.v + b.w*a.v + a.v.cross(b.v),
                        a.w*b.w - a.v.dot(b.v));
}

Vector3f
Quaternionf::rotate (const Vector3f& v_in) const
{
    // Rotation of a vector by q is equivalent to conjugation by q,
    //    q * v * q^{-1}  (where q^{-1} is the same as q's conjugate for unit quaternions).
    // Note that this is certainly not the most efficient way to implement this:
    return ((*this) * Quaternionf(v_in, 0.0f) * (*this).conjugate()).v;
}

Quaternionf 
Quaternionf::fromAxisAngle (const Vector3f axis, const float angle)
{
    const float halfAngle = angle / 2.0f;
    return Quaternionf (axis.normalized() * sin(halfAngle), cos(halfAngle));
}

std::ostream& 
operator<< (std::ostream& os, const Quaternionf& rhs)
{
    os << rhs.v << " " << rhs.w;
    return os;
}

void 
Transformf::invert()
{
    const Quaternionf invRot = rotation.conjugate();
    rotation = invRot;
    translation = -(invRot.rotate(translation));
}

Transformf 
operator* (const Transformf& lhs, const Transformf& rhs)
{
    // [ R_1 t_1 ] * [ R_2 t_2 ] = [ R_1*R_2  R_1*t_2 + t_1 ]
    // [  0   1  ]   [  0   1  ]   [    0           1       ]
    return Transformf (lhs.rotation * rhs.rotation, 
                       lhs.translation + lhs.rotation.rotate(rhs.translation));
}

Vector3f 
operator* (const Transformf& lhs, const Vector3f& rhs)
{
    return lhs.translation + lhs.rotation.rotate (rhs);
}

Matrix4f::Matrix4f(Quaternionf q, Vector3f t)
{
    // Convert a Quaternion into a rotation matrix
    // http://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions#Rotation_matrix_.E2.86.94_quaternion
    data[0] = 1 - 2*q.v.y*q.v.y - 2*q.v.z*q.v.z;
    data[1] = 2*q.v.x*q.v.y + 2*q.v.z*q.w;
    data[2] = 2*q.v.x*q.v.z - 2*q.v.y*q.w;
    data[3] = 0;
    data[4] = 2*q.v.x*q.v.y - 2*q.v.z*q.w;
    data[5] = 1 - 2*q.v.x*q.v.x - 2*q.v.z*q.v.z;
    data[6] = 2*q.v.y*q.v.z + 2*q.v.x*q.w;
    data[7] = 0;
    data[8] = 2*q.v.x*q.v.z + 2*q.v.y*q.w;
    data[9] = 2*q.v.y*q.v.z - 2*q.v.x*q.w;
    data[10] = 1 - 2*q.v.x*q.v.x - 2*q.v.y*q.v.y;
    data[11] = 0;
    data[12] = t.x;
    data[13] = t.y;
    data[14] = t.z;
    data[15] = 1;
}

const char* handToString(Hand hand)
{
    switch (hand) {
    case LEFT_HAND: return "LEFT";
    case RIGHT_HAND: return "RIGHT";
    case BOTH_HANDS: return "BOTH";
    case INVALID_HAND:
    default:
        return "INVALID";
    }
}

Hand stringToHand(const std::string& str)
{
    if (str == "LEFT") return LEFT_HAND;
    if (str == "RIGHT") return RIGHT_HAND;
    if (str == "BOTH") return BOTH_HANDS;
    return INVALID_HAND;
}

class TokenStream
{
public:
    typedef std::string::size_type size_type;
    typedef std::string::value_type value_type;

    TokenStream (const std::string& str);
    ~TokenStream();

    // Each of these returns false if they can't parse the expected type:
    bool getInteger (int& value);
    bool getString (std::string& value);
    bool getFloat (float& value);

    bool getQuaternionf (Quaternionf& value);
    bool getVector3f (Vector3f& value);

    // Expects to find the passed-in string as the next token;
    // returns false if not found:
    bool expectedString (const char* str);

private:
    // Tells if something is whitespace using the current locale:
    bool isWhitespace (const value_type c) const;

    // Advances _curPos to eat up any whitespace between tokens:
    void eatWhitespace();

    const std::string& _line;
    const size_type _endPos;
    size_type _curPos;

    // Everything needs to be parsed in the "C" locale to ensure
    // consistency across platforms and nationalities:
    const LocaleType _locale;
};

TokenStream::TokenStream (const std::string& str)
    : _line(str), 
      _endPos(str.size()), 
      _curPos(0),
#ifdef WIN32
      _locale (_create_locale (LC_NUMERIC, "C"))
#else
      _locale (newlocale (LC_NUMERIC_MASK, "C", LC_GLOBAL_LOCALE))
#endif
{
    eatWhitespace();
}

TokenStream::~TokenStream()
{
#ifdef WIN32
    _free_locale (_locale);
#else
    freelocale (_locale);
#endif
}

bool 
TokenStream::isWhitespace (const value_type c) const
{
#ifdef WIN32
    return (_isspace_l (c, _locale) != 0);
#else
    return (isspace_l (c, _locale) != 0);
#endif
}

void
TokenStream::eatWhitespace()
{
    while (_curPos != _endPos && isWhitespace (_line[_curPos]))
        ++_curPos;
}

bool
TokenStream::getString (std::string& value)
{
    if (_curPos == _endPos)
        return false;

    const size_t start = _curPos;
    while (_curPos != _endPos && !isWhitespace (_line[_curPos]))
        ++_curPos;
    const size_t end = _curPos;

    eatWhitespace ();

    value = std::string (_line.begin() + start, _line.begin() + end);
    return true;
}

bool
TokenStream::getFloat (float& value)
{
    if (_curPos == _endPos)
        return false;

    const char* linePtr = _line.c_str();
    const char* startPtr = linePtr + _curPos;
    char* endPtr;

#ifdef WIN32
    const double result = _strtod_l (startPtr, &endPtr, _locale);
#else
    const double result = strtod_l (startPtr, &endPtr, _locale);
#endif

    // Failed to parse:
    if (endPtr == nullptr)
        return false;

    _curPos = (endPtr - linePtr);

    // Parsing terminated prematurely:
    if (_curPos != _endPos && !isWhitespace(_line[_curPos]))
        return false;

    eatWhitespace();

    value = static_cast<float> (result);
    return true;
}

bool
TokenStream::getInteger (int& value)
{
    if (_curPos == _endPos)
        return false;

    const char* linePtr = _line.c_str();
    const char* startPtr = linePtr + _curPos;
    char* endPtr;

#ifdef WIN32
    const int64_t result = _strtoi64_l (startPtr, &endPtr, 10, _locale);
#else
    const int64_t result = strtoll_l (startPtr, &endPtr, 10, _locale);
#endif

    // Failed to parse:
    if (endPtr == nullptr)
        return false;

    _curPos = (endPtr - linePtr);

    // Parsing terminated prematurely:
    if (_curPos != _endPos && !isWhitespace(_line[_curPos]))
        return false;

    eatWhitespace();

    if (result > static_cast<int64_t>(INT_MAX) || result < static_cast<int64_t>(INT_MIN))
        return false;

    value = static_cast<int> (result);
    return true;
}

bool 
TokenStream::expectedString (const char* str)
{
    std::string foundStr;
    if (!getString(foundStr))
        return false;

    return foundStr == str;
}

bool 
TokenStream::getQuaternionf (Quaternionf& value)
{
    if (!getFloat(value.v.x)) return false;
    if (!getFloat(value.v.y)) return false;
    if (!getFloat(value.v.z)) return false;
    if (!getFloat(value.w))   return false;
    return true;
}

bool 
TokenStream::getVector3f (Vector3f& value)
{
    if (!getFloat(value.x)) return false;
    if (!getFloat(value.y)) return false;
    if (!getFloat(value.z)) return false;
    return true;
}

BasicMessage*           parseBasicMessage         (TokenStream& stream, HandTrackingMessage::MessageType messageType);
WelcomeMessage*         parseWelcomeMessage       (TokenStream& stream, HandTrackingMessage::MessageType messageType);
UserMessage*            parseUserMessage          (TokenStream& stream, HandTrackingMessage::MessageType messageType);
PoseMessage*            parsePoseMessage          (TokenStream& stream, HandTrackingMessage::MessageType messageType);
PinchMessage*           parsePinchMessage         (TokenStream& stream, HandTrackingMessage::MessageType messageType);
BimanualPinchMessage*   parseBimanualPinchMessage (TokenStream& stream, HandTrackingMessage::MessageType messageType);
PointMessage*           parsePointMessage         (TokenStream& stream, HandTrackingMessage::MessageType messageType);

HandTrackingMessage*
HandTrackingMessage::deserialize(const std::string& data)
{
    TokenStream stream (data);

    std::string messageTypeStr;
    if (!stream.getString(messageTypeStr))
        return nullptr;  // No message (blank line)

    MessageType messageType = stringToMessageType(messageTypeStr);
    switch (messageType)
    {
    case WELCOME:
        return parseWelcomeMessage (stream, messageType);
    case USER:
        return parseUserMessage (stream, messageType);
    case POSE:
        return parsePoseMessage (stream, messageType);
    case PRESSED:
    case DRAGGED:
    case RELEASED:
    case MOVED:
        return parsePinchMessage (stream, messageType);
    case SIMULTANEOUSLY_PRESSED:
    case INDIVIDUALLY_PRESSED:
    case SIMULTANEOUSLY_RELEASED:
    case INDIVIDUALLY_RELEASED:
    case DRAGGED_BIMANUAL:
        return parseBimanualPinchMessage (stream, messageType);
    case POINT:
        return parsePointMessage (stream, messageType);
    default:
        return nullptr;
    }
}

const char* 
HandTrackingMessage::messageTypeToString( MessageType m )
{
    switch (m)
    {
    case WELCOME: return "WELCOME";
    case USER: return "USER";
    case POSE: return "POSE";
    case PRESSED: return "PRESSED";
    case DRAGGED: return "DRAGGED";
    case RELEASED: return "RELEASED";
    case MOVED: return "MOVED";
    case SIMULTANEOUSLY_PRESSED: return "SIMULTANEOUSLY_PRESSED";
    case INDIVIDUALLY_PRESSED: return "INDIVIDUALLY_PRESSED";
    case SIMULTANEOUSLY_RELEASED: return "SIMULTANEOUSLY_RELEASED";
    case INDIVIDUALLY_RELEASED: return "INDIVIDUALLY_RELEASED";
    case DRAGGED_BIMANUAL: return "DRAGGED_BIMANUAL";
    case POINT: return "POINT";
    default: return "INVALID_DATA";
    };
}

HandTrackingMessage::MessageType 
HandTrackingMessage::stringToMessageType( const std::string& str )
{
    if (str == "WELCOME") return WELCOME;
    if (str == "USER") return USER;
    if (str == "POSE") return POSE;
    if (str == "PRESSED") return PRESSED;
    if (str == "DRAGGED") return DRAGGED;
    if (str == "RELEASED") return RELEASED;
    if (str == "MOVED") return MOVED;
    if (str == "SIMULTANEOUSLY_PRESSED") return SIMULTANEOUSLY_PRESSED;
    if (str == "INDIVIDUALLY_PRESSED") return INDIVIDUALLY_PRESSED;
    if (str == "SIMULTANEOUSLY_RELEASED") return SIMULTANEOUSLY_RELEASED;
    if (str == "INDIVIDUALLY_RELEASED") return INDIVIDUALLY_RELEASED;
    if (str == "DRAGGED_BIMANUAL") return DRAGGED_BIMANUAL;
    if (str == "POINT") return POINT;
    return INVALID_DATA;
}

HandTrackingMessage::~HandTrackingMessage() { }

BasicMessage*
parseBasicMessage (TokenStream& stream, HandTrackingMessage::MessageType messageType)
{
    std::array<Vector3f,N_HANDS> positions;
    std::array<Quaternionf,N_HANDS> rotations;
    std::array<int,N_HANDS> clickCount;

    // Parse the hand state of each hand
    for (int iHand=0; iHand<N_HANDS; iHand++)
    {
        if (!stream.getVector3f(positions[iHand])) throw ParseException ("Error parsing BasicMessage: invalid position.");
        if (!stream.getQuaternionf(rotations[iHand])) throw ParseException ("Error parsing BasicMessage: invalid position.");

        if (!stream.getInteger (clickCount[iHand])) throw ParseException ("Error parsing BasicMessage: invalid click count.");
        if (clickCount[iHand] < 0 || clickCount[iHand] > 2)
            throw ParseException ("Error parsing BasicMessage: invalid click count.");
    }

    return new BasicMessage(messageType, 
                            positions[0], rotations[0], clickCount[0], 
                            positions[1], rotations[1], clickCount[1]);
}

BasicMessage::BasicMessage( 
    const MessageType type, 
    const Vector3f& positionLeft, 
    const Quaternionf& rotationLeft, 
    const int clickCountLeft, 
    const Vector3f& positionRight, 
    const Quaternionf& rotationRight, 
    const int clickCountRight) 
    : _type(type)
{
    _hands[0] = HandState(positionLeft, rotationLeft, clickCountLeft);
    _hands[1] = HandState(positionRight, rotationRight, clickCountRight);
} 

std::string BasicMessage::serialize() const
{
    std::ostringstream buffer;
    buffer.imbue (std::locale("C")); // Listeners assume en-US locale for numbers

    buffer << messageTypeToString(_type);
    for (size_t iHand = 0; iHand < N_HANDS; ++iHand)
        buffer << " " << _hands[iHand].getPosition() << " " << _hands[iHand].getRotation() << " " << _hands[iHand].getClickCount();
    return buffer.str();
}

BasicMessage::~BasicMessage() { }

PinchMessage::PinchMessage( 
    const MessageType type, 
    const Hand hand,
    const Vector3f& positionLeft, 
    const Quaternionf& rotationLeft, 
    const int clickCountLeft, 
    const Vector3f& positionRight, 
    const Quaternionf& rotationRight, 
    const int clickCountRight) 
    : BasicMessage(type, positionLeft, rotationLeft, clickCountLeft, 
                         positionRight, rotationRight, clickCountRight),
      _hand(hand) { } 

std::string PinchMessage::serialize() const
{
    std::ostringstream buffer;
    buffer.imbue (std::locale("C")); // Listeners assume en-US locale for numbers

    buffer << BasicMessage::serialize();
    buffer << " " << handToString(_hand);
    return buffer.str();
}

PinchMessage*
parsePinchMessage (TokenStream& stream, HandTrackingMessage::MessageType messageType)
{
    std::auto_ptr<BasicMessage> basicMessage (parseBasicMessage (stream, messageType));
    const HandState left = basicMessage->getHandState(0);
    const HandState right = basicMessage->getHandState(1);

    std::string handStr;
    if (!stream.getString(handStr))
        throw ParseException ("Error parsing PinchMessage: no hand found."); 

    return new PinchMessage (messageType,
                             stringToHand(handStr),
                             left.getPosition(),
                             left.getRotation(),
                             left.getClickCount(),
                             right.getPosition(),
                             right.getRotation(),
                             right.getClickCount());
}

BimanualPinchMessage::BimanualPinchMessage( 
    const MessageType type, 
    const Hand hand,
    const Vector3f& positionLeft, 
    const Quaternionf& rotationLeft, 
    const int clickCountLeft, 
    const Vector3f& positionRight, 
    const Quaternionf& rotationRight, 
    const int clickCountRight) 
    : BasicMessage(type, positionLeft, rotationLeft, clickCountLeft, 
                         positionRight, rotationRight, clickCountRight),
      _hand(hand) { } 

std::string 
BimanualPinchMessage::serialize() const
{
    std::ostringstream buffer;
    buffer.imbue (std::locale("C")); // Listeners assume en-US locale for numbers

    buffer << BasicMessage::serialize();
    buffer << " " << handToString(_hand);
    return buffer.str();
}

BimanualPinchMessage*
parseBimanualPinchMessage (TokenStream& stream, HandTrackingMessage::MessageType messageType)
{
    std::auto_ptr<BasicMessage> basicMessage (parseBasicMessage (stream, messageType));
    const HandState left = basicMessage->getHandState(0);
    const HandState right = basicMessage->getHandState(1);

    std::string handStr;
    if (!stream.getString(handStr))
        throw ParseException ("Error parsing BimanualPinchMessage: no hand found."); 

    return new BimanualPinchMessage(messageType,
                                    stringToHand(handStr),
                                    left.getPosition(),
                                    left.getRotation(),
                                    left.getClickCount(),
                                    right.getPosition(),
                                    right.getRotation(),
                                    right.getClickCount());
}

PointMessage::PointMessage(const Hand hand,
                           const Vector3f& pointDir,
                           const Vector3f& pointEnd,
                           const float confidence)
    : _hand (hand),
      _pointDir (pointDir),
      _pointEnd (pointEnd),
      _confidence (confidence)
{
}

std::string 
PointMessage::serialize() const 
{
    const char* handName = handToString(_hand);
    const char* messageTypeName = messageTypeToString(getType());
    assert (handName != nullptr);
    assert (messageTypeName != nullptr);
    std::ostringstream buffer;
    buffer.imbue (std::locale("C")); // Listeners assume en-US locale for numbers

    buffer << messageTypeName << " " << handName
        << " " << _pointDir
        << " " << _pointEnd
        << " " << _confidence;
    return buffer.str();
}

PointMessage*
parsePointMessage (TokenStream& stream, HandTrackingMessage::MessageType messageType)
{
    std::string handStr;
    if (!stream.getString (handStr))
        throw ParseException ("Error parsing PointMessage: no hand found.");

    const Hand hand = stringToHand(handStr);
    if (hand == INVALID_HAND) 
        throw ParseException("Error parsing PointMessage: invalid HAND.");

    Vector3f pointDir;
    if (!stream.getVector3f (pointDir))  throw ParseException("Error parsing PointMessage: invalid pointing direction.");

    Vector3f pointEnd;
    if (!stream.getVector3f (pointEnd)) throw ParseException("Error parsing PointMessage: invalid pointing endpoint.");

    float confidence;
    if (!stream.getFloat (confidence)) throw ParseException("Error parsing PointMessage: invalid confidence value.");

    if (confidence < 0.0f || confidence > 1.0f)
        throw ParseException("Error parsing PointMessage: invalid confidence value.");

    return new PointMessage(hand, pointDir, pointEnd, confidence);
}

PoseMessage::PoseMessage(const Vector3f& positionLeft,
                         const Quaternionf& rotationLeft,
                         const int clickCountLeft,
                         const Vector3f& positionRight,
                         const Quaternionf& rotationRight,
                         const int clickCountRight,
                         const std::array<float, N_HANDS>& confidenceEstimates,
                         const std::array<std::array<Quaternionf, N_JOINTS>, N_HANDS>& jointRotations,
                         const std::array<std::array<Vector3f, N_JOINTS>, N_HANDS>& jointTranslations,
                         const std::array<std::array<Vector3f, N_FINGERS>, N_HANDS>& fingerTips,
                         const std::array<std::array<float, N_POSES>, N_HANDS>& handPoseConfidences )
                         : BasicMessage(HandTrackingMessage::POSE, 
                                        positionLeft, rotationLeft, clickCountLeft, 
                                        positionRight, rotationRight, clickCountRight),
                           _confidenceEstimates(confidenceEstimates),
                           _jointRotations(jointRotations),
                           _jointTranslations(jointTranslations),
                           _fingerTips(fingerTips),
                           _handPoseConfidences(handPoseConfidences) { }

std::string 
PoseMessage::serialize() const 
{
    std::ostringstream buffer;
    buffer.imbue (std::locale("C")); // Listeners assume en-US locale for numbers

    buffer << BasicMessage::serialize();
    for (int iHand=0; iHand<N_HANDS; iHand++) 
    {
        buffer << " " << _confidenceEstimates[iHand];
        for (int jJoint=0; jJoint<N_JOINTS; jJoint++)
        {
            buffer << " " << _jointRotations[iHand][jJoint];
            buffer << " " << _jointTranslations[iHand][jJoint];
        }
        for (int jFinger=0; jFinger<N_FINGERS; jFinger++)
        {
            buffer << " " << _fingerTips[iHand][jFinger];
        }
    }

    for (int iHand=0; iHand<N_HANDS; iHand++) 
    {
        for(int jPose = 0; jPose < N_POSES; jPose++)
        {
            buffer << " " << _handPoseConfidences[iHand][jPose];
        }
    }

    return buffer.str();
}

PoseMessage*
parsePoseMessage (TokenStream& stream, HandTrackingMessage::MessageType messageType)
{
    std::auto_ptr<BasicMessage> basicMessage (parseBasicMessage (stream, messageType));

    const HandState left = basicMessage->getHandState(0);
    const HandState right = basicMessage->getHandState(1);

    std::array<float,N_HANDS> confidenceEstimates;
    std::array<std::array<Quaternionf, N_JOINTS>, N_HANDS> jointRotations;
    std::array<std::array<Vector3f, N_JOINTS>, N_HANDS> jointTranslations;
    std::array<std::array<Vector3f, N_FINGERS>, N_HANDS> fingerTips;
    std::array<std::array<float, N_POSES>, N_HANDS> handPoseConfidences;

    for (int iHand=0; iHand<N_HANDS; iHand++)
    {
        if (!stream.getFloat (confidenceEstimates[iHand])) throw ParseException ("Error parsing PoseMessage: invalid confidence.");

        if (confidenceEstimates[iHand] < 0.0f || confidenceEstimates[iHand] > 1.0f)
            throw ParseException ("Error parsing PoseMessage: invalid confidence.");

        for (int jJoint=0; jJoint<N_JOINTS; jJoint++) 
        {
            if (!stream.getQuaternionf (jointRotations[iHand][jJoint])) throw ParseException ("Error parsing PoseMessage: invalid joint rotation.");
            if (!stream.getVector3f (jointTranslations[iHand][jJoint])) throw ParseException ("Error parsing PoseMessage: invalid joint translation.");
        }

        for (int jFingerTip=0; jFingerTip < N_FINGERS; jFingerTip++) 
        {
            if (!stream.getVector3f (fingerTips[iHand][jFingerTip])) throw ParseException ("Error parsing PoseMessage: invalid finger tips.");
        }
    }

    for (int iHand=0; iHand<N_HANDS; iHand++)
    {
        for (int jPose=0; jPose<N_POSES; jPose++) 
        {
            if (!stream.getFloat (handPoseConfidences[iHand][jPose])) throw ParseException ("Error parsing PoseMessage: invalid pose confidence estimates.");
        }
    }

    return new PoseMessage (left.getPosition(),
                            left.getRotation(),
                            left.getClickCount(),
                            right.getPosition(),
                            right.getRotation(),
                            right.getClickCount(),
                            confidenceEstimates,
                            jointRotations,
                            jointTranslations,
                            fingerTips,
                            handPoseConfidences);
}

std::array<Matrix4f, N_JOINTS> 
PoseMessage::getJointFrames( size_t hand ) const
{
    std::array<Matrix4f, N_JOINTS> transforms;

    for (int jJoint=0; jJoint < N_JOINTS; jJoint++)
    {
        transforms[jJoint] = Matrix4f(_jointRotations[hand][jJoint], _jointTranslations[hand][jJoint]);
    }
    return transforms;
}

std::array<Transformf, N_JOINTS> 
PoseMessage::getJointTransforms( size_t hand ) const
{
    std::array<Transformf, N_JOINTS> transforms;

    for (int jJoint=0; jJoint < N_JOINTS; jJoint++)
    {
        transforms[jJoint] = Transformf(_jointRotations[hand][jJoint], _jointTranslations[hand][jJoint]);
    }
    return transforms;
}

WelcomeMessage::WelcomeMessage(const std::string& serverVersion, 
                               const std::string& protocolVersion) :
    _serverVersion(serverVersion), _protocolVersion(protocolVersion) { }

std::string
WelcomeMessage::serialize() const
{
    std::ostringstream buffer;
    buffer.imbue (std::locale("C")); // Listeners assume en-US locale for numbers

    buffer << messageTypeToString(getType());
    buffer << " Server-Version: " << _serverVersion;
    buffer << " Protocol-Version: " << _protocolVersion;
    return buffer.str();
}

WelcomeMessage*
parseWelcomeMessage (TokenStream& stream, HandTrackingMessage::MessageType messageType)
{
    if (!stream.expectedString ("Server-Version:"))
        throw ParseException ("Error parsing welcome message: expected 'Server-Version:'.");
    std::string serverVersion;
    if (!stream.getString (serverVersion))
        throw ParseException ("Error parsing welcome message: expected server version.");

    if (!stream.expectedString ("Protocol-Version:"))
        throw ParseException ("Error parsing welcome message: expected 'Protocol-Version:'.");
    std::string protocolVersion;
    if (!stream.getString (protocolVersion))
        throw ParseException ("Error parsing welcome message: expected protocol version.");

    return new WelcomeMessage(serverVersion, protocolVersion);
}

UserMessage::UserMessage(const std::string& userProfileName, 
                         const std::array<std::vector<Vector3f>, N_HANDS>& restPositions,
                         const std::array<std::vector<Triangle>, N_HANDS>& triangles,
                         const std::array<std::vector<IndicesVector>, N_HANDS>& skinningIndices,
                         const std::array<std::vector<WeightsVector>, N_HANDS>& skinningWeights,
                         const std::array<std::array<Quaternionf, N_JOINTS>, N_HANDS>& restJointRotations,
                         const std::array<std::array<Vector3f, N_JOINTS>, N_HANDS>& restJointTranslations)
                         : _userProfileName(userProfileName),
                           _restPositions(restPositions),
                           _triangles(triangles),
                           _skinningIndices(skinningIndices),
                           _skinningWeights(skinningWeights),
                           _restJointRotations(restJointRotations),
                           _restJointTranslations(restJointTranslations) { }

std::string 
UserMessage::serialize() const
{
    std::ostringstream buffer;
    buffer.imbue (std::locale("C")); // Listeners assume en-US locale for numbers

    buffer << messageTypeToString(getType());
    buffer << " User: " << _userProfileName;
    for (int iHand=0; iHand<N_HANDS; iHand++)
    {
        buffer << " Hand: " << iHand;
        buffer << " Rest-Positions: " << _restPositions[iHand].size();
        for (size_t v=0; v<_restPositions[iHand].size(); v++) 
        {
            buffer << " " << _restPositions[iHand][v];
        }

        buffer << " Triangles: " << _triangles[iHand].size();
        for (size_t jTriangle=0; jTriangle<_triangles[iHand].size(); jTriangle++)
        {
            Triangle p = _triangles[iHand][jTriangle];
            buffer << " " << p[0] << " " << p[1] << " " << p[2];
        }

        buffer << " Skinning-Weights:";
        for (size_t i=0; i<_skinningIndices[iHand].size(); i++) 
        {
            IndicesVector indices = _skinningIndices[iHand][i];
            WeightsVector weights = _skinningWeights[iHand][i];

            buffer << " " << indices.size();

            for (size_t j=0; j<indices.size(); j++) 
            {
                buffer << " ";
                buffer << indices[j];
                buffer << " ";
                buffer << weights[j];
            }
        }

        buffer << " Rest-Joint-Frames:";
        for (size_t jJoint=0; jJoint<N_JOINTS; jJoint++)
        {
            buffer << " " << _restJointRotations[iHand][jJoint];
            buffer << " " << _restJointTranslations[iHand][jJoint];
        }
    }
    
    return buffer.str();
}

UserMessage*
parseUserMessage (TokenStream& stream, HandTrackingMessage::MessageType messageType)
{
    if (!stream.expectedString ("User:"))
        throw ParseException ("Error parsing User message: missing 'User'.");

    std::string userProfileName;
    if (!stream.getString(userProfileName))
        throw ParseException ("Error parsing User message: missing profile name.");

    typedef UserMessage::Triangle Triangle;
    typedef UserMessage::IndicesVector IndicesVector;
    typedef UserMessage::WeightsVector WeightsVector;

    std::array<std::vector<Vector3f>, N_HANDS> restPositions;
    std::array<std::vector<Triangle>, N_HANDS> triangles;
    std::array<std::vector<IndicesVector>, N_HANDS> skinningIndices;
    std::array<std::vector<WeightsVector>, N_HANDS> skinningWeights;
    std::array<std::array<Quaternionf, N_JOINTS>, N_HANDS> restJointRotations;
    std::array<std::array<Vector3f, N_JOINTS>, N_HANDS> restJointTranslations;

    const int TOO_MANY = 100000;  // Just a sanity check on the vertex count
                                  // to make sure if we get bad data we don't try to allocate
                                  // an infinite-sized array.

    for (int iHand=0; iHand<N_HANDS; iHand++) 
    {
        if (!stream.expectedString ("Hand:"))
            throw ParseException ("Error parsing User message: missing 'Hand'");

        int hand;
        if (!stream.getInteger (hand)) throw ParseException ("Error parsing User message: invalid hand index.");

        if (hand != iHand)
            throw ParseException ("Error parsing User message: invalid hand index.");

        if (!stream.expectedString ("Rest-Positions:"))
            throw ParseException ("Error parsing User message: missing 'Rest-Positions:'");

        int nVertices;
        if (!stream.getInteger (nVertices)) throw ParseException ("Error parsing User message: unable to parse vertex count.");

        if (nVertices <= 0 || nVertices > TOO_MANY)
            throw ParseException ("Error parsing User message: invalid vertex count.");

        restPositions[iHand].resize(nVertices);
        skinningIndices[iHand].resize(nVertices);
        skinningWeights[iHand].resize(nVertices);

        for (size_t i=0; i<restPositions[iHand].size(); i++) 
        {
            if (!stream.getVector3f (restPositions[iHand][i])) throw ParseException ("Error parsing User message: invalid rest positions.");
        }

        if (!stream.expectedString ("Triangles:"))
            throw ParseException ("Error parsing User message: missing 'Triangles:'");

        int nTriangles;
        if (!stream.getInteger (nTriangles)) throw ParseException ("Error parsing User message: unable to parse vertex count.");

        if (nTriangles <= 0 || nTriangles > TOO_MANY)
            throw ParseException ("Error parsing User message: invalid triangle count.");;

        triangles[iHand].resize(nTriangles);

        for (int i=0; i<triangles[iHand].size(); i++) 
        {
            for (int j=0; j<triangles[iHand][i].size(); j++) 
            {
                int idx;
                if (!stream.getInteger (idx)) throw ParseException ("Error parsing User message: unable to parse vertex index.");

                if (idx < 0 || idx >= nVertices)
                    throw ParseException ("Error parsing User message: invalid vertex index.");

                triangles[iHand][i][j] = idx;
            }
        }

        if (!stream.expectedString ("Skinning-Weights:"))
            throw ParseException ("Error parsing User message: missing 'Skinning-Weights:'");

        for (int i=0; i<skinningIndices[iHand].size(); i++) 
        {
            int nInfluences;
            if (!stream.getInteger (nInfluences)) throw ParseException ("Error parsing User message: unable to parse joint influence count.");

            if (nInfluences < 0 || nInfluences > 10)
                throw ParseException ("Error parsing User message: too many joint influences.");

            skinningIndices[iHand][i].resize(nInfluences);
            skinningWeights[iHand][i].resize(nInfluences);
            for (int j=0; j<nInfluences; j++) 
            {
                int bone;
                if (!stream.getInteger (bone)) throw ParseException ("Error parsing User message: unable to read joint influence bone.");
                if (bone < 0 || bone >= N_JOINTS)
                    throw ParseException ("Error parsing User message: invalid joint influence bone.");

                float weight;
                if (!stream.getFloat (weight)) throw ParseException ("Error parsing User message: unable to read joint influence weight.");
                
                if (weight < 0.0f || weight > 1.0f)
                    throw ParseException ("Error parsing User message: invalid joint influence weight.");

                skinningIndices[iHand][i][j] = bone;
                skinningWeights[iHand][i][j] = weight;
            }
        }

        if (!stream.expectedString ("Rest-Joint-Frames:"))
            throw ParseException ("Error parsing User message: missing 'Rest-Joint-Frames:'");

        for (int jJoint=0; jJoint<N_JOINTS; jJoint++)
        {
            if (!stream.getQuaternionf (restJointRotations[iHand][jJoint])) throw ParseException ("Error parsing User message: invalid rest joint rotation.");
            if (!stream.getVector3f (restJointTranslations[iHand][jJoint])) throw ParseException ("Error parsing User message: invalid rest joint translation.");
        }
    }

    return new UserMessage (userProfileName, 
                            restPositions, 
                            triangles, 
                            skinningIndices, 
                            skinningWeights, 
                            restJointRotations, 
                            restJointTranslations);
}

std::array<Matrix4f, N_JOINTS> 
UserMessage::getRestJointFrames( int hand ) const
{
    std::array<Matrix4f, N_JOINTS> jointFrames;
    for (int jJoint=0; jJoint<N_JOINTS; jJoint++)
    {
        jointFrames[jJoint] = Matrix4f(_restJointRotations[hand][jJoint], 
                                       _restJointTranslations[hand][jJoint]);
    }
    return jointFrames;
}

std::array<Transformf, N_JOINTS> 
UserMessage::getRestJointTransforms( int hand ) const
{
    std::array<Transformf, N_JOINTS> jointFrames;
    for (int jJoint=0; jJoint<N_JOINTS; jJoint++)
    {
        jointFrames[jJoint] = Transformf(_restJointRotations[hand][jJoint], 
                                         _restJointTranslations[hand][jJoint]);
    }
    return jointFrames;
}

} // namespace HandTrackingClient
