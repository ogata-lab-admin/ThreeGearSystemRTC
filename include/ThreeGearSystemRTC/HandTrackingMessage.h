#pragma once

#include <string>
#include <vector>
#include <array>
#include <iosfwd>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace HandTrackingClient
{

const static int N_JOINTS = 17;
const static int N_FINGERS = 5;
const static int N_HANDS = 2;
const static int N_POSES = 7;

//! \brief A minimal 3-vector class.  
//!
//! We have provided this simple vector class to
//! avoid a dependence on an external math library
//! (like Eigen). 
class Vector3f 
{
public:
    //! The x component.
    float x;

    //! The y component.
    float y;

    //! The z component.
    float z;

    //! The default constructor initializes to (0, 0, 0).
    Vector3f() : x(0), y(0), z(0) {}

    //! Initialize from the passed-in values.
    Vector3f(float x_in, float y_in, float z_in) : x(x_in), y(y_in), z(z_in) {}

    //! Adds the passed-in vector to this vector.
    Vector3f& operator+= (const Vector3f& rhs)     { x += rhs.x; y += rhs.y; z += rhs.z; return *this; }

    //! Subtracts the passed-in vector to this vector.
    Vector3f& operator-= (const Vector3f& rhs)     { x -= rhs.x; y -= rhs.y; z -= rhs.z; return *this; }

    //! Unary minus operator.
    Vector3f operator-() const                     { return Vector3f(-x, -y, -z); }

    //! Multiplies this vector (in place) by the scalar.
    Vector3f& operator*= (const float rhs)         { x *= rhs; y *= rhs; z *= rhs; return *this; }

    //! Divides this vector (in place) by the scalar.
    Vector3f& operator/= (const float rhs)         { x /= rhs; y /= rhs; z /= rhs; return *this; }

    //! Computes the dot product of this vector with another.
    float dot (const Vector3f& rhs) const          { return x*rhs.x + y*rhs.y + z*rhs.z; }

    //! Computes the cross product of this vector with another.
    Vector3f cross (const Vector3f& rhs) const;

    //! Tests for equality (no floating point slop).
    bool operator== (const Vector3f& rhs) const    { return (x == rhs.x) && (y == rhs.y) && (z == rhs.z); }

    //! Tests for inequality (no floating point slop).
    bool operator!= (const Vector3f& rhs) const    { return !(*this == rhs); }

    //! Returns the L2 squared norm (x^2 + y^2 + z^2).
    float squaredNorm() const                      { return x*x + y*y + z*z; }

    //! Returns the l_infinity norm (the supremum over the absolute values of the elements).  
    float lInfNorm() const                         { return (std::max) ((std::max) (std::abs(x), std::abs(y)), std::abs(z)); }

    //! Returns the L2 norm sqrt(x^2 + y^2 + z^2).
    float norm() const                             { return std::sqrt(squaredNorm()); }

    //! Normalize this vector.  Note that it avoids the divide by zero
    //! in cases where the vector is equal to (0, 0, 0).
    void normalize();

    //! Returns a normalized version of this vector.
    Vector3f normalized() const                    { Vector3f result(*this); result.normalize(); return result; }
};

//! Multiplies a vector by a scalar.
inline Vector3f operator* (const float lhs, const Vector3f& rhs)           { Vector3f result(rhs); result *= lhs; return result; }

//! Multiplies a vector by a scalar.
inline Vector3f operator* (const Vector3f& lhs, const float rhs)           { Vector3f result(lhs); result *= rhs; return result; }

//! Divides a vector by a scalar.
inline Vector3f operator/ (const Vector3f& lhs, const float rhs)           { Vector3f result(lhs); result /= rhs; return result; }

//! Adds two vectors together.
inline Vector3f operator+ (const Vector3f& lhs, const Vector3f& rhs)       { Vector3f result(lhs); result += rhs; return result; }

//! Subtracts the second vector from the first.
inline Vector3f operator- (const Vector3f& lhs, const Vector3f& rhs)       { Vector3f result(lhs); result -= rhs; return result; }

//! Prints a vector to the output stream in the format "v.x v.y v.z".
std::ostream& operator<< (std::ostream& os, const Vector3f& rhs);

//! \brief A minimal quaternion class.  
//!
//! We have provided this simple Quaternion class to
//! avoid a dependence on an external math library
//! (like Eigen). 
class Quaternionf 
{
public:
    //! The vector component
    Vector3f v;

    //! The w scalar component.
    float w;

    //! Initializes the quaternion to the identity rotation.
    Quaternionf() : v(0, 0, 0), w(1) {}

    //! Initializes the quaternion from the passed-in values.
    Quaternionf(float x_in, float y_in, float z_in, float w_in) 
        : v(x_in, y_in, z_in), w(w_in) {}

    //! \brief Initializes the quaternion from vector and scalar components.
    //! 
    //! Note that this is _not_ the same as initializing with an axis-angle
    //! pair (see Quaternionf::fromAxisAngle).  
    //!
    //! @param v_in  Vector part.
    //! @param w_in  Scalar part.
    Quaternionf(const Vector3f& v_in, float w_in)
        : v(v_in), w(w_in) {}

    //! Initializes the quaternion with the passed-in axis/angle pair.  
    //!
    //! @param axis  Rotation axis, must be normalized.
    //! @param angle Angle in radians.
    static Quaternionf fromAxisAngle (const Vector3f axis, const float angle);

    //! \brief Computes and returns the quaternion conjugate.  
    //! 
    //! For normalized quaternions (which represent rotations), the conjugate
    //! represents the inverse rotation.
    Quaternionf conjugate() const;

    //! Rotates the passed-in vector by the rotation represented by this quaternion.  
    Vector3f rotate (const Vector3f& v) const;

    //! Normalizes this quaternion.  
    void normalize();

    //! Returns a normalized version of this quaternion.   
    Quaternionf normalized() const { Quaternionf result (*this); result.normalize(); return result; }

    //! Tests for equality (no floating point slop).
    bool operator== (const Quaternionf& rhs) const    { return (v == rhs.v) && (w == rhs.w); }

    //! Tests for inequality (no floating point slop).
    bool operator!= (const Quaternionf& rhs) const    { return !(*this == rhs); }
};

//! Multiplies two quaternions.
Quaternionf operator* (const Quaternionf& a, const Quaternionf& b);

//! Prints a quaternion to the output stream in the format "v.x v.y v.z w"
std::ostream& operator<< (std::ostream& os, const Quaternionf& rhs);

/*! Represents the transform
    \f[
      \left[ \begin{array}{cc}
         \mathbf{R} & \mathbf{t} \\
         \mathbf{0} & 1
      \end{array} \right]
    \f]
    The final transformation is \f$ T(\mathbf{X}) = \mathbf{R}*\mathbf{x} + \mathbf{t} \f$, where \f$\mathbf{R}\f$ is the
    rotational component and \f$ \mathbf{t} \f$ is the translational component.
*/
class Transformf
{
public:
    //! This is the rotation component of the transform.  
    Quaternionf rotation;

    //! This is the translation component of the transform.
    Vector3f translation;

    //! Constructs a transform from the given rotation and translation.
    Transformf (const Quaternionf& rotation_in, const Vector3f& translation_in)
        : rotation (rotation_in), translation (translation_in) {}

    Transformf() {}

    //! Inverts this transform.
    void invert();

    //! Returns the inverse of this transform.
    Transformf inverse() const { Transformf result(*this); result.invert(); return result; }
};

Transformf operator* (const Transformf& lhs, const Transformf& rhs);
Vector3f operator* (const Transformf& lhs, const Vector3f& rhs);

//! 4x4 matrix stored in column major format.
//!
//! We have provided this simple Matrix4f class to
//! avoid a dependence on an external math library
//! (like Eigen).  
class Matrix4f 
{
public:
    //! The 4x4 = 16 matrix entries, stored column-major.  
    float data[16];

    //! Initializes with the matrix of all 0s.
    Matrix4f() { std::fill(data, data+16, 0.0f); }

    //! Initializes with the upper-left 3x3 block set to the
    //! rotation specified by q and the translation component
    //! set to t.  
    Matrix4f(Quaternionf q, Vector3f t);
};

//! \brief Structure holding the position, rotational frame and other information
//!        relevant to the state of the hand.
//!
//! This is the _overall_ position and rotation estimate for the hand.  
//! It is used to pass around location information for clicks.  Users
//! needing more detailed information about individual finger joints
//! should look at the PoseMessage.  
class HandState
{
public:
    //! Constructs a hand state with the given position and rotation.  
    HandState(const Vector3f& position, 
              const Quaternionf& rotation, 
              const int clickCount) 
        : _position(position), 
          _rotation(rotation),
          _clickCount(clickCount) { }

    //! Constructs a hand state with position = (0,0,0) and the identity rotation.  
    HandState() { }

    //! \brief Position estimate for the hand.  
    //!
    //! This is defined by the rigid frame given by the back of
    //! the hand, so it will be considerably more stable than
    //! the individual finger position estimates given by the PoseMessage.
    const Vector3f& getPosition() const { return _position; }

    //! Overall orientation estimate for the hand.  
    const Quaternionf& getRotation() const { return _rotation; }

    //! Returns 1 if the hand is single clicked, 2 if the hand is
    //! double-clicked, and 0 if neither.  
    const int getClickCount() const { return _clickCount; }

private:
    Vector3f _position;
    Quaternionf _rotation;
    int _clickCount;
};

//! Most events (like click events) only apply to
//! one hand at a time, but simultaneous click and
//! release events will have the BOTH_HANDS identifier set.  
enum Hand
{
    LEFT_HAND = 0,        //! Message applies only to the left hand.
    RIGHT_HAND = 1,       //! Message applies only to the right hand.
    BOTH_HANDS = 2,       //! Message applies to both hands (e.g. a simultaneous click).  
    INVALID_HAND = 10000  //! You should never see this in callback messages; it is used internally when network deserialization fails.  
};

//! Returns a string representation for the Hand enum.
//! Used for serializing and deserializing from the 
//! network.  
const char* handToString(Hand hand);

//! Given a string ("left" or "right"), returns the
//! correct Hand.  
//! Used for serializing and deserializing from the 
//! network.  
Hand stringToHand(const std::string& str);

//! Exception thrown when parsing a message fails.  
class ParseException
    : public std::exception
{
public:
    ParseException (const char* message)
        : _msg (message) {}
    virtual ~ParseException() {}
    
    virtual const char* what() const throw() { return _msg; }

private:
    const char* _msg;
};

///
/// Base class for all hand-tracking messages / events. 
///
class HandTrackingMessage
{
public:
    //! An enum describing the message type.  
    //! Used for distinguishing between similar messages, e.g., between
    //! PRESSED and RELEASED, which are both PinchMessages.  
    enum MessageType  
    {
        WELCOME = 0,                    //! Provides server and protocol versions
        USER = 100,                     //! Provides information about the user, such as profile name and skinning information
        POSE = 200,                     //! Provides full skeleton pose information
        PRESSED = 300,                  //! User pinched index and thumb together
        DRAGGED = 301,                  //! User moved hand while holding a pinch
        RELEASED = 302,                 //! User released the pinch
        MOVED = 303,                    //! User moved without pinching
        SIMULTANEOUSLY_PRESSED = 400,   //! User pinched thumb and index finger of both hands at (approximately) the same time
        INDIVIDUALLY_PRESSED = 401,     //! User pinched thumb and index finger of one hand (compare to SIMULTANEOUSLY_PRESSED)
        SIMULTANEOUSLY_RELEASED = 402,  //! User released thumb and index finger of both hands at the same time
        INDIVIDUALLY_RELEASED = 403,    //! User released thumb and index finger of one hand (compare to SIMULTANEOUSLY_RELEASED)
        DRAGGED_BIMANUAL = 404,         //! User moved hands while both were pinching
        POINT = 500,                    //! User pointed at something with the index finger
        INVALID_DATA = 10000            //! This should never happen; in practice it will be ignored
    };

    //! Convert from a MessageType to its string representation; used for serialization.  
    static const char* messageTypeToString(MessageType m);

    //! Convert from a string to a MessageType; used in deserialization.  
    static MessageType stringToMessageType(const std::string& str);

    //! \brief Given a line of text sent over the network connection, 
    //!        converts it to a valid hand message.
    //!
    //! Callers are responsible for freeing the memory.  
    static HandTrackingMessage* deserialize(const std::string& data);

    //! Returns the message type (e.g. PRESSED or DRAGGED).  
    virtual MessageType getType() const = 0 ;

    //! \brief Serializes the message to send over the network.  
    //! 
    //! Developers should not need to worry about this function 
    //! unless they write their own networking code.  
    virtual std::string serialize() const = 0;

    virtual ~HandTrackingMessage();
protected:
    //! Constructor is protected; HandTrackingMessage should only
    //! be constructed from network data using the deserialize() function. 
    HandTrackingMessage() { }
};

//! The basic form of the message includes a type, which hand was updated,
//! and the position and rotational frame of each hand. 
class BasicMessage : public HandTrackingMessage
{
public:
    MessageType getType() const { return _type; }

    //! \brief Returns the overall general hand state, excluding fingers.
    //!
    //! This is the most rigid, stable frame returned by the hand tracking
    //! and is probably the one you want to use for selection.  It is defined by
    //! the position and rotation of the back of the hand (excluding the wrist).  
    const HandState& getHandState(int hand) const { return _hands[hand]; }

    //! Constructs a BasicMessage.  Should not generally be called by users of the API.  
    BasicMessage(const MessageType type, 
                 const Vector3f& positionLeft,
                 const Quaternionf& rotationLeft,
                 const int clickCountLeft,
                 const Vector3f& positionRight,
                 const Quaternionf& rotationRight,
                 const int clickCountRight);
    virtual ~BasicMessage();

    //! Serializes the message to a string, which could then be output
    //! over the network. 
    std::string serialize() const;

private:
    MessageType _type;
    std::array<HandState, N_HANDS> _hands;
};

//! \brief Messages relating to pressing, releasing, dragging and moving of each hand.
//!
//! Pinch messages are always a single hand at a time.  That is, if 
//! the user pinches left and right simultaneously, you will still get 
//! two separate PRESS messages (one for the right and one for the left).
//! As a result, there is less lag for pinch messages (which can be reported
//! as soon as they happen) than for the "higher-level" \ref BimanualPinchMessage
//! messages.  
//!
//! Note that we do not recommend responding to both the PinchMessage
//! and the \ref BimanualPinchMessage; applications should generally
//! pick one or the other.  Responding to both can lead to very 
//! confusing interactions (since most pinches will be detected at least
//! twice).  
class PinchMessage : public BasicMessage
{
public:
    //! Constructs a PinchMessage.  Should not generally be called by users of the API.  
    PinchMessage(const MessageType type, 
                 const Hand hand,
                 const Vector3f& positionLeft,
                 const Quaternionf& rotationLeft,
                 const int clickCountLeft,
                 const Vector3f& positionRight,
                 const Quaternionf& rotationRight,
                 const int clickCountRight);

    //! @return which hand(s) this message applies to
    const Hand getHand() const { return _hand; }

    std::string serialize() const;

private:
    Hand _hand;
};

//! \brief Messages relating to simultaneous or individual pinching.
//!
//! Bimanual pinch messages are a ``higher-level'' message than the regular
//! pinch message, and can be used to distinguish whether one hand pinched
//! or whether both pinched simultaneously.  If the user presses both hands
//! at the same time, for example, you will only get a single Bimanual
//! SIMULTANEOUSLY_PRESSED message (instead of two PRESSED messages, as you
//! would with the PinchMessage). The cost
//! of this higher-level knowledge is a bit of additional lag: to determine
//! whether two pinches happen at the same time we have to wait 100ms or so 
//! (since the pinches are never going to be _exactly_ simultaneous).  You 
//! can decide whether the added functionality is worth the lag and choose which
//! of the two interfaces you prefer.  
//!
//! Note that we do not recommend responding to both the BimanualPinchMessage
//! and the \ref PinchMessage; applications should generally
//! pick one or the other.  Responding to both can lead to very 
//! confusing interactions (since most pinches will be detected at least
//! twice).  
//! 
//! To help understanding the difference between the PinchMessage and the
//! BimanualPinchMessage, consider the following two scenarios:
//!
//! First, suppose the user presses her left hand and then, a second later,
//! her right hand.  You will receive the following sequence of events:
//!
//!    \li PinchMessage(PRESSED, left hand)
//!    \li ...
//!    \li BimanualPinchMessage(INDIVIDUALLY_PRESSED, left hand)
//!    \li ...
//!    \li PinchMessage(PRESSED, right hand)
//!    \li ...
//!    \li BimanualPinchMessage(INDIVIDUALLY_PRESSED, right hand)
//!
//! Note how the BimanualPinchMessage always lags slightly behind the 
//! PinchMessage (you will receive it a couple frames later).  
//! Now, suppose the user presses her left and right hands simultaneously.
//!
//!    \li PinchMessage(PRESSED, left hand)
//!    \li PinchMessage(PRESSED, right hand)
//!    \li BimanualPinchMessage(SIMULTANEOUSLY_PRESSED, both hands)
//! In the case of the double pinch, it can be reported immediately. 
class BimanualPinchMessage : public BasicMessage
{
public:
    //! Constructs a BimanualPinchMessage.  Should not generally be called by users of the API.  
    BimanualPinchMessage(const MessageType type, 
                         const Hand hand,
                         const Vector3f& positionLeft,
                         const Quaternionf& rotationLeft,
                         const int clickCountLeft,
                         const Vector3f& positionRight,
                         const Quaternionf& rotationRight,
                         const int clickCountRight);

    //! @return which hand(s) this message applies to
    const Hand getHand() const { return _hand; }

    std::string serialize() const;

private:
    Hand _hand;
};

///
/// Messages relating to pointing
///
class PointMessage : public HandTrackingMessage
{
public:
    //! Constructs a PointMessage.  Should not generally be called by users of the API.  
    PointMessage(const Hand hand,
                 const Vector3f& pointDir,
                 const Vector3f& pointEnd,
                 const float confidence);

    //! @return which hand(s) this message applies to
    const Hand getHand() const { return _hand; }

    std::string serialize() const;

    virtual MessageType getType() const { return HandTrackingMessage::POINT; }

    //! This is the direction the finger is pointing in.  
    Vector3f getPointDir() const { return _pointDir; }

    //! This is our estimate of the end of the finger.  
    Vector3f getPointEnd() const { return _pointEnd; }

    //! This is the confidence that the user is actually pointing at something.  
    float getConfidence() const  { return _confidence; }

private:
    Hand _hand;
    Vector3f _pointDir;
    Vector3f _pointEnd;
    float _confidence;
};

///
/// Messages giving the full hand pose (all joint angles)
///
class PoseMessage : public BasicMessage
{

public:
    //! An enum distinguishing between the seven current pose types that are recognized by
    //! the hand tracking.
    enum HandPose
    {
        HAND_CURLED = 0,       //! The hand curled up, ready to pinch
        HAND_ELL = 1,          //! The sign language "L" shape
        HAND_OKAY = 2,         //! The "okay" symbol; thumb and index touching and other three fingers spread
        HAND_PINCH = 3,        //! Thumb and forefinger touching
        HAND_POINTING = 4,     //! Index finger point
        HAND_RELAXEDOPEN = 5,  //! What we consider a "neutral" pose, with fingers slightly curled
        HAND_SPREAD = 6        //! All five fingers spread
    };

    //! Constructs a PoseMessage.  Should not generally be called by users of the API.  
    PoseMessage(const Vector3f& positionLeft,
                const Quaternionf& rotationLeft,
                const int clickCountLeft,
                const Vector3f& positionRight,
                const Quaternionf& rotationRight,
                const int clickCountRight,
                const std::array<float, N_HANDS>& confidenceEstimates,
                const std::array<std::array<Quaternionf, N_JOINTS>, N_HANDS>& jointRotations,
                const std::array<std::array<Vector3f, N_JOINTS>, N_HANDS>& jointTranslations,
                const std::array<std::array<Vector3f, N_FINGERS>, N_HANDS>& fingerTips,
                const std::array<std::array<float, N_POSES>, N_HANDS>& handPoseConfidences);

    std::string serialize() const;

    //! \brief How confident we are about the pose.  
    //!
    //! This is a value between 0 and 1.  Currently only values of 
    //! zero (0) and one (1) are ever returned, but we expect this
    //! to change in future versions.
    float getConfidenceEstimate(size_t hand) const { return _confidenceEstimates[hand]; }

    //! The joint frames of all the bones; used for skinning. 
    std::array<Matrix4f, N_JOINTS> getJointFrames(size_t hand) const;
    std::array<Transformf, N_JOINTS> getJointTransforms(size_t hand) const;

    //! The points at the ends of the fingers.  
    const std::array<Vector3f, N_FINGERS>& getFingerTips(size_t hand) const { return _fingerTips[hand]; }

    //! \brief Experimental function that returns how confident we are that the hand is in a given pose.
    //!
    //! Returns the confidence score for each pose.  Each confidence score
    //! is a value between 0 and 1 and they sum to 1.  Please use the
    //! the PoseMessage::HandPose enum to index into the array.
    const std::array<float, N_POSES>& getHandPoseConfidences(size_t hand) const { return _handPoseConfidences[hand]; }

private:
    std::array<float, N_HANDS> _confidenceEstimates;
    std::array<std::array<Quaternionf, N_JOINTS>, N_HANDS> _jointRotations;
    std::array<std::array<Vector3f, N_JOINTS>, N_HANDS> _jointTranslations;
    std::array<std::array<Vector3f, N_FINGERS>, N_HANDS> _fingerTips;
    std::array<std::array<float, N_POSES>, N_HANDS> _handPoseConfidences;
};

///
/// A message sent upon connecting to the server, indicating the server and
/// protocol version.
///
class WelcomeMessage : public HandTrackingMessage
{
public:
    //! Constructs a WelcomeMessage.  Should not generally be called by users of the API.  
    WelcomeMessage(const std::string& serverVersion, const std::string& protocolVersion);

    MessageType getType() const { return HandTrackingMessage::WELCOME; }

    //! Current version of the hand tracking server.  
    const std::string getServerVersion() const { return _serverVersion; }

    //! Current version of the protocol (should change infrequently), 
    //! could be used to detect incompatibility.  
    const std::string getProtocolVersion() const { return _protocolVersion; }

    std::string serialize() const;

private:
    std::string _serverVersion;
    std::string _protocolVersion;
};

//! \brief Message that exposes the user name and skinning information for the user's
//!        calibrated hands.
//!
//! This message will be sent every time the user profile changes.
//! This will happen every time a new client connects, but also if
//! the scale of the user's hands appears to change.  
//!
//! You can use this data to display an active cursor of the user's hand.  
//! For client applications, however, we have moved away from displaying
//! the whole skinned hand (which can be distracting) in favor of more 
//! abstract cursors.  
//!
//! The skinning used by our system is "Linear blend skinning"
//! also known as "Smooth skinning."
//! 
//! http://graphics.ucsd.edu/courses/cse169_w05/3-Skin.htm
//!
class UserMessage : public HandTrackingMessage 
{
public:
    //! A triangle is just 3 ints (one for each vertex).  
    typedef std::array<int,3> Triangle;

    // A list of bone influence indices (for a single vertex).
    typedef std::vector<int> IndicesVector;

    // A list of bone influence weights (for a single vertex).
    typedef std::vector<float> WeightsVector;

    //! The name of the user's profile.
    const std::string& getUserProfileName() const { return _userProfileName; };

    //! \brief The positions of the hand in its rest pose (with the bones
    //! given by getRestJointFrames(), below).  
    const std::vector<Vector3f>& getRestPositions(int hand) const { return _restPositions[hand]; } 

    //! The topology of the skinned hand mesh.  
    const std::vector<Triangle>& getTriangles(int hand) const { return _triangles[hand]; }

    //! Skinning indices for the hand.  See \ref getSkinningWeights for an example of how this is used.  
    const std::vector<IndicesVector>& getSkinningIndices(int hand) const { return _skinningIndices[hand]; }

    //! \brief Skinning weights for the hand.  
    //! 
    //! Given a UserMessage and a JointMessage, here is how to generate a skinned
    //! hand model (using linear blend skinning):
    //! \code{.cpp}
    //! UserMessage* userMessage = ...;   // Received once when the profile is set; needs to be cached.
    //! PoseMessage* poseMessage = ...;   // Received once per tracking frame.
    //! for (size_t iHand = 0; iHand < NUMBER_OF_HANDS; ++iHand)
    //! {
    //!     const std::vector<IndicesVector>& skinningIndices = 
    //!         userMessage->getSkinningIndices(iHand);
    //!     const std::vector<WeightsVector>& skinningWeights = 
    //!         userMessage->getWeightsVector(iHand);
    //!     const std::vector<Vector3f>& restPositions = 
    //!         userMessage->getRestPositions(iHand);
    //!     const std::array<Matrix4f, N_JOINTS> currentPoseTransforms = 
    //!         poseMessage->getJointFrames(iHand);
    //!     const std::array<Matrix4f, N_JOINTS> restPoseTransforms = 
    //!         userMessage->getRestJointFrames(iHand);
    //!     
    //!     // The rest points are relative to the rest transforms,
    //!     // so to get the final points we need to _invert_ the
    //!     // the rest transforms before applying the current pose.
    //!     std::array<Matrix4f, N_JOINTS> relativeTransforms;
    //!     for (size_t j = 0; j < N_JOINTS; ++j)
    //!         relativeTransforms[i] = currentPose[j] * restPoseTransforms[j].inverse();
    //!
    //!     std::vector<Vector3f> skinnedPoints = restPositions;
    //!     const size_t nVertices = skinningIndices.size();
    //!     for (size_t jVertex = 0; jVertex != nVertices; ++jVertex)
    //!     {
    //!         // Influence bones/weights for _this_ vertex:
    //!         const IndicesVector& influenceBones = skinningIndices[jVertex];
    //!         const WeightsVector& influenceWeights = skinningWeights[jVertex];
    //!         const size_t nInfluences = influenceBones.size();
    //!         assert (influenceWeights.size() == nInfluences);
    //!
    //!         Vector3f skinnedPosition (0, 0, 0);
    //!         for (size_t kInfluence = 0; kInfluence != nInfluences; ++kInfluence)
    //!         {
    //!             const float w = influenceWeights[kInfluence];
    //!             const int b = influenceBones[kInfluence];
    //!             skinnedPosition += w * (relativeTransforms[b] * restPositions[kVertex]);
    //!         }
    //!         skinnedPoints[jVertex] = skinnedPosition;
    //!     }
    //! }
    //! \endcode
    const std::vector<WeightsVector>& getSkinningWeights(int hand) const { return _skinningWeights[hand]; }

    //! The joint frames of the hand in its rest pose.  
    std::array<Matrix4f, N_JOINTS> getRestJointFrames(int hand) const;
    std::array<Transformf, N_JOINTS> getRestJointTransforms(int hand) const; 

    //! Constructs a UserMessage.  Should not generally be called by users of the API.  
    UserMessage(const std::string& userProfileName, 
                const std::array<std::vector<Vector3f>, N_HANDS>& restPositions,
                const std::array<std::vector<Triangle>, N_HANDS>& triangles,
                const std::array<std::vector<IndicesVector>, N_HANDS>& skinningIndices,
                const std::array<std::vector<WeightsVector>, N_HANDS>& skinningWeights,
                const std::array<std::array<Quaternionf, N_JOINTS>, N_HANDS>& restJointRotations,
                const std::array<std::array<Vector3f, N_JOINTS>, N_HANDS>& restJointTranslations);

    MessageType getType() const { return HandTrackingMessage::USER; }

    std::string serialize() const;

private:
    std::string _userProfileName;
    std::array<std::vector<Vector3f>, N_HANDS> _restPositions;
    std::array<std::vector<Triangle>, N_HANDS> _triangles;
    std::array<std::vector<IndicesVector>, N_HANDS> _skinningIndices;
    std::array<std::vector<WeightsVector>, N_HANDS> _skinningWeights;
    std::array<std::array<Quaternionf, N_JOINTS>, N_HANDS> _restJointRotations;
    std::array<std::array<Vector3f, N_JOINTS>, N_HANDS> _restJointTranslations;
};

} // namespace HandTrackingClient

