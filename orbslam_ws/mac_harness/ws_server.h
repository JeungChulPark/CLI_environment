// Minimal HTTP + WebSocket server for the live viewer: serves the files of one directory and
// pushes messages to every connected browser. Push only (RFC 6455, unmasked server frames);
// what a browser sends is read solely to notice that it went away.
#pragma once

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <openssl/evp.h>
#include <openssl/sha.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <functional>
#include <iostream>
#include <list>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using Msg = std::shared_ptr<const std::string>;  // one complete, already framed WebSocket message

inline Msg ws_frame(const std::string& payload, bool binary) {
  std::string f;
  const uint64_t n = payload.size();
  f.reserve(n + 10);
  f.push_back(char(binary ? 0x82 : 0x81));
  if (n < 126) {
    f.push_back(char(n));
  } else if (n < 65536) {
    f.push_back(char(126)); f.push_back(char(n >> 8)); f.push_back(char(n & 0xff));
  } else {
    f.push_back(char(127));
    for (int i = 7; i >= 0; i--) f.push_back(char((n >> (8 * i)) & 0xff));
  }
  f += payload;
  return std::make_shared<const std::string>(std::move(f));
}

class LiveServer {
 public:
  // on_connect returns what a newly connected browser needs to catch up with the run so far
  LiveServer(std::string web_dir, std::function<std::vector<Msg>()> on_connect)
      : web_dir_(std::move(web_dir)), on_connect_(std::move(on_connect)) {}

  bool start(const std::string& host, int port) {
    lfd_ = socket(AF_INET, SOCK_STREAM, 0);
    int one = 1;
    setsockopt(lfd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(port);
    if (inet_pton(AF_INET, host.c_str(), &a.sin_addr) != 1) return false;
    if (::bind(lfd_, (sockaddr*)&a, sizeof a) < 0 || listen(lfd_, 16) < 0) {
      perror("live server bind/listen");
      return false;
    }
    std::thread([this] { accept_loop(); }).detach();
    return true;
  }

  // dynamic GET routes, tried before the file directory: return true and fill body to answer
  using Route = std::function<bool(const std::string& path, std::string& body)>;
  void set_route(Route r) { route_ = std::move(r); }

  // kind > 0: at most `keep` messages of that kind wait per browser and older ones are dropped,
  // so a slow browser sees fewer frames rather than falling behind (it never slows the tracker)
  void broadcast(const Msg& m, int kind = 0, size_t keep = 0) {
    std::lock_guard<std::mutex> l(cm_);
    for (auto it = clients_.begin(); it != clients_.end();) {
      auto c = *it;
      if (c->closed) { it = clients_.erase(it); continue; }
      {
        std::lock_guard<std::mutex> cl(c->m);
        if (kind > 0) {
          size_t same = 0;
          for (auto& q : c->q) same += q.kind == kind;
          for (auto qi = c->q.begin(); same >= keep && qi != c->q.end();) {
            if (qi->kind == kind) { qi = c->q.erase(qi); same--; } else ++qi;
          }
        }
        c->q.push_back({m, kind});
      }
      c->cv.notify_one();
      ++it;
    }
  }

  size_t clients() {
    std::lock_guard<std::mutex> l(cm_);
    size_t n = 0;
    for (auto& c : clients_) n += !c->closed;
    return n;
  }

 private:
  struct Item { Msg m; int kind; };
  struct Client {
    int fd = -1;
    std::mutex m;
    std::condition_variable cv;
    std::deque<Item> q;
    std::atomic<bool> closed{false};
    ~Client() { close(fd); }  // the reader and writer threads both hold a reference
    void mark_closed() {
      { std::lock_guard<std::mutex> l(m); closed = true; }
      shutdown(fd, SHUT_RDWR);
      cv.notify_all();
    }
  };

  static bool send_all(int fd, const std::string& s) {
    size_t off = 0;
    while (off < s.size()) {
      ssize_t k = send(fd, s.data() + off, s.size() - off, 0);
      if (k <= 0) return false;
      off += k;
    }
    return true;
  }

  void accept_loop() {
    while (true) {
      int fd = accept(lfd_, nullptr, nullptr);
      if (fd < 0) continue;
      int one = 1;
      setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof one);
      setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
      std::thread([this, fd] { handle(fd); }).detach();
    }
  }

  static std::string header(const std::string& req, const std::string& name) {
    std::string low = req;
    for (auto& ch : low) ch = tolower(ch);
    size_t p = low.find("\r\n" + name + ":");
    if (p == std::string::npos) return "";
    p += name.size() + 3;
    size_t e = req.find("\r\n", p);
    std::string v = req.substr(p, e - p);
    v.erase(0, v.find_first_not_of(' '));
    v.erase(v.find_last_not_of(' ') + 1);
    return v;
  }

  void handle(int fd) {
    std::string req;
    char buf[4096];
    while (req.find("\r\n\r\n") == std::string::npos && req.size() < 16384) {
      ssize_t k = recv(fd, buf, sizeof buf, 0);
      if (k <= 0) { close(fd); return; }
      req.append(buf, k);
    }
    std::istringstream rs(req);
    std::string method, path;
    rs >> method >> path;
    std::string key = header(req, "sec-websocket-key");
    if (key.empty()) {
      serve_file(fd, path);
      close(fd);
      return;
    }

    static const std::string guid = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    unsigned char sha[SHA_DIGEST_LENGTH];
    const std::string k = key + guid;
    SHA1((const unsigned char*)k.data(), k.size(), sha);
    unsigned char b64[32];
    EVP_EncodeBlock(b64, sha, SHA_DIGEST_LENGTH);
    if (!send_all(fd, "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                      "Sec-WebSocket-Accept: " + std::string((char*)b64) + "\r\n\r\n")) {
      close(fd);
      return;
    }

    auto c = std::make_shared<Client>();
    c->fd = fd;
    {
      std::lock_guard<std::mutex> l(cm_);  // catch-up and live messages stay in order
      for (auto& m : on_connect_()) c->q.push_back({m, 0});
      clients_.push_back(c);
    }
    std::thread([c] {
      while (true) {
        Item it;
        {
          std::unique_lock<std::mutex> l(c->m);
          c->cv.wait(l, [&] { return !c->q.empty() || c->closed; });
          if (c->closed) return;
          it = std::move(c->q.front());
          c->q.pop_front();
        }
        if (!send_all(c->fd, *it.m)) { c->mark_closed(); return; }
      }
    }).detach();

    // a browser only ever sends close / pong frames; any close or EOF ends the connection
    while (!c->closed) {
      ssize_t n = recv(fd, buf, sizeof buf, 0);
      if (n <= 0 || (buf[0] & 0x0f) == 0x8) break;
    }
    c->mark_closed();
  }

  void serve_file(int fd, std::string path) {
    path = path.substr(0, path.find('?'));
    if (path == "/") path = "/index.html";
    std::string body;
    if (!(route_ && route_(path, body))) {
      std::ifstream f;
      if (path.find("..") == std::string::npos) f.open(web_dir_ + path, std::ios::binary);
      if (!f) {
        send_all(fd, "HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n");
        return;
      }
      body.assign((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    }
    auto ends = [&](const char* e) {
      std::string s(e);
      return path.size() >= s.size() && path.compare(path.size() - s.size(), s.size(), s) == 0;
    };
    const char* type = ends(".html") ? "text/html; charset=utf-8"
                     : ends(".js")   ? "text/javascript"
                     : ends(".css")  ? "text/css"
                                     : "application/octet-stream";
    send_all(fd, "HTTP/1.1 200 OK\r\nContent-Type: " + std::string(type) + "\r\nContent-Length: " +
                     std::to_string(body.size()) + "\r\nCache-Control: no-store\r\nAccess-Control-Allow-Origin: *\r\nConnection: close\r\n\r\n" + body);
  }

  std::string web_dir_;
  std::function<std::vector<Msg>()> on_connect_;
  Route route_;
  int lfd_ = -1;
  std::mutex cm_;
  std::list<std::shared_ptr<Client>> clients_;
};
