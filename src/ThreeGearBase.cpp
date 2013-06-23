#include "ThreeGearBase.h"

Position::Position(){
	this->x = 0;
	this->y = 0;
	this->z = 0;
}

Position::~Position(){}

void Position::SetPosition(float x, float y, float z){
	this->x = x;
	this->y = y;
	this->z = z;
}

float Position::GetPositionX(){ return this->x; }

float Position::GetPositionY(){ return this->y; }

float Position::GetPositionZ(){ return this->z; }

Rotation::Rotation(){
	this->w = 0;
	this->v_x = 0;
	this->v_y = 0;
	this->v_x = 0;
}

Rotation::~Rotation(){}

float Rotation::GetRotationW(){ return this->w; }

float Rotation::GetRotationV_X(){ return this->v_x; }

float Rotation::GetRotationV_Y(){ return this->v_y; }

float Rotation::GetRotationV_Z(){ return this->v_z; }

ThreeGearBase::ThreeGearBase(){
	Position handPosition();
	Rotation handRotatins();

	_client.addHandTrackingListener(this);
	std::pair<bool, std::string> result= _client.connect();
	if(!result.first){
		std::string errMsg = "Unable to connect to hand Tracking server: ";
		errMsg += result.second;
		std::cout << "connection error: " << errMsg << std::endl;
	}
}

ThreeGearBase::~ThreeGearBase(){}


void ThreeGearBase::handleEvent(const HandTrackingClient::HandTrackingMessage& baseMessage){
	using HandTrackingClient::HandTrackingMessage;
	using HandTrackingClient::PinchMessage;

	if(baseMessage.getType() == HandTrackingMessage::MOVED){
		const PinchMessage& message = dynamic_cast<const PinchMessage&>(baseMessage);
		HandTrackingClient::HandState state = message.getHandState(message.getHand());
		this->SetPosition(state.getPosition().x, state.getPosition().y, state.getPosition().z);
		this->SetRotation(state.getRotation().w, state.getRotation().v.x, state.getRotation().v.y, state.getRotation().v.z);
	}
	
}


