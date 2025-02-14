// E57PointCloudWriter.h
#ifndef E57_POINTCLOUD_WRITER_H
#define E57_POINTCLOUD_WRITER_H

#include <cstdint>
#include <memory>
#include <string>

struct Point {
    double x, y, z;
    uint8_t r, g, b;
    double intensity;
    double gpstime;
};

// 
class E57PointCloudWriter {
public:
    E57PointCloudWriter();
    ~E57PointCloudWriter();

    bool open(const std::string& strSavePath);
    void writePoint(const Point& pt);
    void close();

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
    
    // 禁用拷贝构造和赋值
    E57PointCloudWriter(const E57PointCloudWriter&) = delete;
    E57PointCloudWriter& operator=(const E57PointCloudWriter&) = delete;
};

#endif
