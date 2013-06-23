#include "HandTrackingClient.h"
#include "HandTrackingListener.h"
#include <iostream>

class Position{
	private:
		float x;
		float y;
		float z;
	public:
		Position();
		~Position();
		void SetPosition(float x, float y, float z);
		float GetPositionX();
		float GetPositionY();
		float GetPositionZ();
	};

class Rotation{
	private:
		float w;
		float v_x;
		float v_y;
		float v_z;
	public:
		Rotation();
		~Rotation();
		void SetRotation(float w, float v_x, float v_y, float v_z);
		float GetRotationW();
		float GetRotationV_X();
		float GetRotationV_Y();
		float GetRotationV_Z();
};


class ThreeGearBase : public HandTrackingClient::HandTrackingListener, public Position, public Rotation{
private:
	HandTrackingClient::Client _client;
public:
	virtual void handleEvent(const HandTrackingClient::HandTrackingMessage& message);
	virtual void handleConnectionClosed(){ };
	ThreeGearBase(void);
	~ThreeGearBase(void);
};