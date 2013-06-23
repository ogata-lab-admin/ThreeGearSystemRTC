#include "HandTrackingClient.h"
#include "Threads.h"

#include <iostream>
#include <asio.hpp>

namespace HandTrackingClient
{

class ClientImpl 
{
public:
    ClientImpl() : _stopClientThread(false) { }
    ~ClientImpl();

    std::pair<bool, std::string> connect(const uint16_t port = 1988);
    void addHandTrackingListener(HandTrackingListener* listener);
    void stop();

private:
    // Runs in a new thread
    void listen();
    HandTrackingListener* _listener;

    std::unique_ptr<Thread> _clientThread;
    bool _stopClientThread;

    std::auto_ptr<asio::io_service> _ioService;
    std::auto_ptr<asio::ip::tcp::socket> _clientSocket;
};

Client::Client() { _clientImpl.reset(new ClientImpl); }

Client::~Client() { }

std::pair<bool, std::string> Client::connect(const uint16_t port)
{
    return _clientImpl->connect(port);
}

void Client::addHandTrackingListener(HandTrackingListener* listener)
{
    _clientImpl->addHandTrackingListener(listener);
}

void Client::stop()
{
    _clientImpl->stop();
}

ClientImpl::~ClientImpl()
{
    stop();
}

std::pair<bool, std::string> 
ClientImpl::connect(const uint16_t port)
{
    _ioService.reset(new asio::io_service);

    using asio::ip::tcp;
    tcp::endpoint endpoint(
        asio::ip::address::from_string("127.0.0.1"), port);
    _clientSocket.reset(new tcp::socket(*_ioService));
    asio::error_code error = asio::error::host_not_found;
    _clientSocket->connect(endpoint, error);

    if(error)
        return std::make_pair(false, error.message());

    // Function object that calls the io_service::run method:
    class ListenFunctor
    {
    public:
        ListenFunctor (ClientImpl& impl) : _clientImpl(impl) {}
        void operator()() { _clientImpl.listen(); }

    private:
        ClientImpl& _clientImpl;
    };

    // Start a new thread to connect to the hand tracking server
    _clientThread.reset(
        new Thread(ListenFunctor(*this)));

    return std::make_pair(true, std::string());
}

void ClientImpl::stop()
{
    _stopClientThread = true;
    if (_clientThread.get() != nullptr) 
    {
        _clientThread->join();
        _clientThread.reset();
    }
}

void ClientImpl::addHandTrackingListener(HandTrackingListener* listener)
{
    _listener = listener;
}

void ClientImpl::listen()
{
    asio::streambuf receiveBuffer;
    std::string resultString; 

    while(!_stopClientThread)
    {
        asio::error_code error;
        // Read one message every time
        asio::read_until(*_clientSocket, receiveBuffer, "\n", error);
        if(error) break;
        std::istream is(&receiveBuffer);
        resultString.clear();
        std::getline(is, resultString);
        is.unsetf(std::ios_base::skipws);

        try
        {
            std::auto_ptr<HandTrackingMessage> msg (
                HandTrackingMessage::deserialize(resultString));
            // Ignore messages we don't understand
            if (msg.get() == nullptr) continue;
            // Parse received messages from the hand tracking server 
            _listener->handleEvent(*msg);
        }
        catch (std::exception& e)
        {
            std::cerr << "Error parsing message: " << e.what() << "\n";
        }
    }

    _listener->handleConnectionClosed();
}

} // namespace HandTrackingAppClient

