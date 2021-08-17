#include "assert.hpp"
#include "client_ws.hpp"
#include "server_ws.hpp"
#include <iostream>

using namespace std;
using namespace SimpleWeb;

class SocketServerTest : public SocketServerBase<WS> {
public:
  SocketServerTest() : SocketServerBase<WS>::SocketServerBase(8080) {}

  void accept() {}

  void parse_request_test() {
    std::shared_ptr<Connection> connection(new Connection(handler_runner, 0, *io_service));

    ostream ss(&connection->streambuf);
    ss << "GET /test/ HTTP/1.1\r\n";
    ss << "TestHeader: test\r\n";
    ss << "TestHeader2:test2\r\n";
    ss << "TestHeader3:test3a\r\n";
    ss << "TestHeader3:test3b\r\n";
    ss << "\r\n";

    std::istream stream(&connection->streambuf);
    ASSERT(RequestMessage::parse(stream, connection->method, connection->path, connection->query_string, connection->http_version, connection->header));

    ASSERT(connection->method == "GET");
    ASSERT(connection->path == "/test/");
    ASSERT(connection->http_version == "1.1");

    ASSERT(connection->header.size() == 4);
    auto header_it = connection->header.find("TestHeader");
    ASSERT(header_it != connection->header.end() && header_it->second == "test");
    header_it = connection->header.find("TestHeader2");
    ASSERT(header_it != connection->header.end() && header_it->second == "test2");

    header_it = connection->header.find("testheader");
    ASSERT(header_it != connection->header.end() && header_it->second == "test");
    header_it = connection->header.find("testheader2");
    ASSERT(header_it != connection->header.end() && header_it->second == "test2");

    auto range = connection->header.equal_range("testheader3");
    auto first = range.first;
    auto second = first;
    ++second;
    ASSERT(range.first != connection->header.end() && range.second != connection->header.end() &&
           ((first->second == "test3a" && second->second == "test3b") ||
            (first->second == "test3b" && second->second == "test3a")));
  }
};

class SocketClientTest : public SocketClientBase<WS> {
public:
  SocketClientTest(const std::string &server_port_path) : SocketClientBase<WS>::SocketClientBase(server_port_path, 80) {}

  void connect() {}

  void parse_response_header_test() {
    auto connection = std::shared_ptr<Connection>(new Connection(handler_runner, config.timeout_idle, *io_service));
    connection->in_message = std::shared_ptr<InMessage>(new InMessage());

    ostream stream(&connection->in_message->streambuf);
    stream << "HTTP/1.1 200 OK\r\n";
    stream << "TestHeader: test\r\n";
    stream << "TestHeader2:test2\r\n";
    stream << "TestHeader3:test3a\r\n";
    stream << "TestHeader3:test3b\r\n";
    stream << "\r\n";

    ASSERT(ResponseMessage::parse(*connection->in_message, connection->http_version, connection->status_code, connection->header));

    ASSERT(connection->header.size() == 4);
    auto header_it = connection->header.find("TestHeader");
    ASSERT(header_it != connection->header.end() && header_it->second == "test");
    header_it = connection->header.find("TestHeader2");
    ASSERT(header_it != connection->header.end() && header_it->second == "test2");

    header_it = connection->header.find("testheader");
    ASSERT(header_it != connection->header.end() && header_it->second == "test");
    header_it = connection->header.find("testheader2");
    ASSERT(header_it != connection->header.end() && header_it->second == "test2");

    auto range = connection->header.equal_range("testheader3");
    auto first = range.first;
    auto second = first;
    ++second;
    ASSERT(range.first != connection->header.end() && range.second != connection->header.end() &&
           ((first->second == "test3a" && second->second == "test3b") ||
            (first->second == "test3b" && second->second == "test3a")));

    connection.reset();
  }
};

int main() {
  ASSERT(case_insensitive_equal("Test", "tesT"));
  ASSERT(case_insensitive_equal("tesT", "test"));
  ASSERT(!case_insensitive_equal("test", "tseT"));
  CaseInsensitiveEqual equal;
  ASSERT(equal("Test", "tesT"));
  ASSERT(equal("tesT", "test"));
  ASSERT(!equal("test", "tset"));
  CaseInsensitiveHash hash;
  ASSERT(hash("Test") == hash("tesT"));
  ASSERT(hash("tesT") == hash("test"));
  ASSERT(hash("test") != hash("tset"));

  SocketServerTest serverTest;
  serverTest.io_service = std::make_shared<io_context>();

  serverTest.parse_request_test();

  {
    SocketClientTest clientTest("test.org:8080/test");
    ASSERT(clientTest.path == "/test");
    ASSERT(clientTest.host == "test.org");
    ASSERT(clientTest.port == 8080);
  }

  {
    SocketClientTest clientTest("test.org/test");
    ASSERT(clientTest.path == "/test");
    ASSERT(clientTest.host == "test.org");
    ASSERT(clientTest.port == 80);
  }

  {
    SocketClientTest clientTest("test.org");
    ASSERT(clientTest.path == "/");
    ASSERT(clientTest.host == "test.org");
    ASSERT(clientTest.port == 80);
  }

  {
    SocketClientTest clientTest("test.org:test");
    ASSERT(clientTest.path == "/");
    ASSERT(clientTest.host == "test.org");
    ASSERT(clientTest.port == 80);
  }

  {
    SocketClientTest clientTest("[::1]");
    ASSERT(clientTest.path == "/");
    ASSERT(clientTest.host == "::1");
    ASSERT(clientTest.port == 80);
  }

  {
    SocketClientTest clientTest("[::1]/test");
    ASSERT(clientTest.path == "/test");
    ASSERT(clientTest.host == "::1");
    ASSERT(clientTest.port == 80);
  }

  {
    SocketClientTest clientTest("[::1]:8080");
    ASSERT(clientTest.path == "/");
    ASSERT(clientTest.host == "::1");
    ASSERT(clientTest.port == 8080);
  }

  {
    SocketClientTest clientTest("[::1]:8080/test");
    ASSERT(clientTest.path == "/test");
    ASSERT(clientTest.host == "::1");
    ASSERT(clientTest.port == 8080);
  }

  {
    SocketClientTest clientTest("[::1]:test/test");
    ASSERT(clientTest.path == "/test");
    ASSERT(clientTest.host == "::1");
    ASSERT(clientTest.port == 80);
  }

  {
    SocketClientTest clientTest("test.org:8080");
    clientTest.io_service = std::make_shared<io_context>();
    ASSERT(clientTest.path == "/");
    ASSERT(clientTest.host == "test.org");
    ASSERT(clientTest.port == 8080);

    clientTest.parse_response_header_test();
  }
}
