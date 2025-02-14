#include "writeE57.h"
// e57 2.3.2
#include <E57Format.h>
//#include <E57Version.h>
#include <vector>
#include <limits>
#include <cmath>
#include <memory>
#include <iostream>
#include <random>

struct E57PointCloudWriter::Impl {
    std::shared_ptr<e57::ImageFile> imageFile;
    std::shared_ptr<e57::CompressedVectorNode> points;

    // 编码器
    std::shared_ptr<e57::CompressedVectorWriter> writer;
    std::vector<e57::SourceDestBuffer> buffers;

    // 数据缓冲区
    std::vector<double> xBuf, yBuf, zBuf;
    std::vector<int16_t> rBuf, gBuf, bBuf;
    std::vector<double> intensityBuf;
    std::vector<double> timeBuf;
    int bufferIndex = 0;

    // 范围记录
    double xMin, xMax;
    double yMin, yMax;
    double zMin, zMax;
    double intensityMin, intensityMax;
    double timeMin, timeMax;
    // 坐标偏移
    double xOffset, yOffset, zOffset;

    bool isOpen = false;
    const size_t bufferSize = 1000;

    Impl() {
        initRanges();
    }

    void updateRanges(const Point& pt) {
        xMin = std::min(xMin, pt.x); xMax = std::max(xMax, pt.x);
        yMin = std::min(yMin, pt.y); yMax = std::max(yMax, pt.y);
        zMin = std::min(zMin, pt.z); zMax = std::max(zMax, pt.z);
        intensityMin = std::min(intensityMin, pt.intensity);
        intensityMax = std::max(intensityMax, pt.intensity);
        timeMin = std::min(timeMin, pt.gpstime);
        timeMax = std::max(timeMax, pt.gpstime);
    }

    void initRanges() {
        xMin = yMin = zMin = std::numeric_limits<double>::max();
        xMax = yMax = zMax = -std::numeric_limits<double>::max();
        intensityMin = timeMin = std::numeric_limits<double>::max();
        intensityMax = timeMax = -std::numeric_limits<double>::max();
        xOffset = yOffset = zOffset = 0.0;
    }

    void flushBuffer() {
        if (!writer || xBuf.empty() || bufferIndex == 0) return;

        // 写入缓冲区数据
        writer->write(bufferIndex);
        //points->writer(buffers).write(bufferIndex);
        //e57::CompressedVectorWriter writer = points->writer(buffers);
        //writer.write(bufferIndex);
        
        // 清空缓冲区
        //xBuf.clear(); yBuf.clear(); zBuf.clear();
        //rBuf.clear(); gBuf.clear(); bBuf.clear();
        //intensityBuf.clear(); timeBuf.clear();
        bufferIndex = 0;
    }

    void close()
    {
        flushBuffer();
        writer->close();
        imageFile->close();
    }
};

E57PointCloudWriter::E57PointCloudWriter() : m_impl(new Impl()) {}

E57PointCloudWriter::~E57PointCloudWriter() {
    close();
}

bool E57PointCloudWriter::open(const std::string& strSavePath) {
    try {
        m_impl->imageFile = std::make_shared<e57::ImageFile>(strSavePath, "w");
        m_impl->imageFile->extensionsAdd("", e57::E57_V1_0_URI);//E57_V1_0_URI,VERSION_1_0_URI
        m_impl->imageFile->root().set("formatName", e57::StringNode(*m_impl->imageFile, "ASTM E57 3D Imaging Data File"));
        m_impl->imageFile->root().set("guid", e57::StringNode(*m_impl->imageFile, "{8DE5883C-36D0-423E-8A21-A476DA14AFDC}"));

        int astmMajor = 1;
        int astmMinor = 0;
        e57::ustring libraryId = "";
        e57::Utilities::getVersions(astmMajor, astmMinor, libraryId);
        m_impl->imageFile->root().set("versionMajor", e57::IntegerNode(*m_impl->imageFile, astmMajor));
        m_impl->imageFile->root().set("versionMinor", e57::IntegerNode(*m_impl->imageFile, astmMinor));
        m_impl->imageFile->root().set("e57LibraryVersion", e57::StringNode(*m_impl->imageFile, libraryId));
        m_impl->imageFile->root().set("coordinateMetadata", e57::StringNode(*m_impl->imageFile, ""));

        e57::StructureNode creationDateTime = e57::StructureNode(*m_impl->imageFile);
        creationDateTime.set("dateTimeValue", e57::FloatNode(*m_impl->imageFile, 0.0));
        creationDateTime.set("isAtomicClockReferenced", e57::IntegerNode(*m_impl->imageFile, 0));
        m_impl->imageFile->root().set("creationDateTime", creationDateTime);

        //m_impl->root = std::make_unique<e57::StructureNode>(m_impl->imageFile->root());

        // 创建数据3D结构
        e57::VectorNode data3D(*m_impl->imageFile, true);
        m_impl->imageFile->root().set("data3D", data3D);

        e57::StructureNode scanNode(*m_impl->imageFile);
        scanNode.set("guid", e57::StringNode(*m_impl->imageFile, "{A545CFC4-5CDE-45D4-9C79-11A37C9557FB}"));
        scanNode.set("name", e57::StringNode(*m_impl->imageFile, "scanPointCloud"));
        scanNode.set("description", e57::StringNode(*m_impl->imageFile, "create by ptsmap"));

        // 创建点云字段
        e57::StructureNode prototype(*m_impl->imageFile);
        prototype.set("cartesianX", e57::FloatNode(*m_impl->imageFile, 0.0, e57::E57_SINGLE));//E57_DOUBLE
        prototype.set("cartesianY", e57::FloatNode(*m_impl->imageFile, 0.0, e57::E57_SINGLE));
        prototype.set("cartesianZ", e57::FloatNode(*m_impl->imageFile, 0.0, e57::E57_SINGLE));
        prototype.set("colorRed", e57::IntegerNode(*m_impl->imageFile, 0, 0, 255));
        prototype.set("colorGreen", e57::IntegerNode(*m_impl->imageFile, 0, 0, 255));
        prototype.set("colorBlue", e57::IntegerNode(*m_impl->imageFile, 0, 0, 255));
        prototype.set("intensity", e57::FloatNode(*m_impl->imageFile, 0.0, e57::E57_SINGLE));
        prototype.set("timeStamp", e57::FloatNode(*m_impl->imageFile, 0.0, e57::E57_DOUBLE));

        m_impl->points = std::make_shared<e57::CompressedVectorNode>(*m_impl->imageFile, prototype,e57::VectorNode(*m_impl->imageFile,true));
        //e57::CompressedVectorNode points = e57::CompressedVectorNode(*m_impl->imageFile, prototype, e57::VectorNode(*m_impl->imageFile, true));
        scanNode.set("points", *m_impl->points);
        data3D.append(scanNode);
        //m_impl->root->set("data3D", data3D);

        m_impl->xBuf.resize(m_impl->bufferSize);
        m_impl->yBuf.resize(m_impl->bufferSize);
        m_impl->zBuf.resize(m_impl->bufferSize);
        m_impl->rBuf.resize(m_impl->bufferSize);
        m_impl->gBuf.resize(m_impl->bufferSize);
        m_impl->bBuf.resize(m_impl->bufferSize);
        m_impl->intensityBuf.resize(m_impl->bufferSize);
        m_impl->timeBuf.resize(m_impl->bufferSize);
        // 创建 SourceDestBuffer
        m_impl->buffers.emplace_back(*m_impl->imageFile, "cartesianX", m_impl->xBuf.data(), m_impl->bufferSize, true,true);
        m_impl->buffers.emplace_back(*m_impl->imageFile, "cartesianY", m_impl->yBuf.data(), m_impl->bufferSize, true, true);
        m_impl->buffers.emplace_back(*m_impl->imageFile, "cartesianZ", m_impl->zBuf.data(), m_impl->bufferSize, true, true);
        m_impl->buffers.emplace_back(*m_impl->imageFile, "colorRed", m_impl->rBuf.data(), m_impl->bufferSize, true, true);
        m_impl->buffers.emplace_back(*m_impl->imageFile, "colorGreen", m_impl->gBuf.data(), m_impl->bufferSize, true, true);
        m_impl->buffers.emplace_back(*m_impl->imageFile, "colorBlue", m_impl->bBuf.data(), m_impl->bufferSize, true, true);
        m_impl->buffers.emplace_back(*m_impl->imageFile, "intensity", m_impl->intensityBuf.data(), m_impl->bufferSize, true, true);
        m_impl->buffers.emplace_back(*m_impl->imageFile, "timeStamp", m_impl->timeBuf.data(), m_impl->bufferSize, true, true);

        // 创建编码器
        m_impl->writer = std::make_shared<e57::CompressedVectorWriter>(m_impl->points->writer(m_impl->buffers));
        
        // 初始化范围
        m_impl->initRanges();
        m_impl->isOpen = true;
        return true;
    }
    catch (const e57::E57Exception& e) {
        std::string str = e57::Utilities::errorCodeToString(e.errorCode());
        return false;
    }
    catch (...) {
        return false;
    }
}

void E57PointCloudWriter::writePoint(const Point& pt) {
    if (!m_impl->isOpen) return;

    m_impl->xBuf[m_impl->bufferIndex] = pt.x;
    m_impl->yBuf[m_impl->bufferIndex] = pt.y;
    m_impl->zBuf[m_impl->bufferIndex] = pt.z;
    m_impl->rBuf[m_impl->bufferIndex] = pt.r;
    m_impl->gBuf[m_impl->bufferIndex] = pt.g;
    m_impl->bBuf[m_impl->bufferIndex] = pt.b;
    m_impl->intensityBuf[m_impl->bufferIndex] = pt.intensity;
    m_impl->timeBuf[m_impl->bufferIndex] = pt.gpstime;
    m_impl->bufferIndex++;

    m_impl->updateRanges(pt);

    if (m_impl->bufferIndex >= m_impl->bufferSize) {
        m_impl->flushBuffer();
    }
}

void E57PointCloudWriter::close() {
    if (!m_impl->isOpen) return;

    try {
        // 更新全局范围信息
        e57::Node n = m_impl->imageFile->root().get("/data3D");
        e57::VectorNode data3D(n);
        e57::StructureNode scanNode = e57::StructureNode(data3D.get(0));

        // 保存 pose 
        e57::StructureNode pose = e57::StructureNode(*m_impl->imageFile);
        scanNode.set("pose", pose);
        e57::StructureNode translation = e57::StructureNode(*m_impl->imageFile);
        translation.set("x", e57::FloatNode(*m_impl->imageFile, m_impl->xOffset));
        translation.set("y", e57::FloatNode(*m_impl->imageFile, m_impl->yOffset));
        translation.set("z", e57::FloatNode(*m_impl->imageFile, m_impl->zOffset));
        pose.set("translation", translation);

        // 保存范围
        e57::StructureNode boxNode = e57::StructureNode(*m_impl->imageFile);
        boxNode.set("xMinimum", e57::FloatNode(*m_impl->imageFile, m_impl->xMin));
        boxNode.set("xMaximum", e57::FloatNode(*m_impl->imageFile, m_impl->xMax));
        boxNode.set("yMinimum", e57::FloatNode(*m_impl->imageFile, m_impl->yMin));
        boxNode.set("yMaximum", e57::FloatNode(*m_impl->imageFile, m_impl->yMax));
        boxNode.set("zMinimum", e57::FloatNode(*m_impl->imageFile, m_impl->zMin));
        boxNode.set("zMaximum", e57::FloatNode(*m_impl->imageFile, m_impl->zMax));
        scanNode.set("cartesianBounds", boxNode);

        e57::StructureNode intensityBox = e57::StructureNode(*m_impl->imageFile);
        intensityBox.set("intensityMinimum", e57::FloatNode(*m_impl->imageFile, m_impl->intensityMin));
        intensityBox.set("intensityMaximum", e57::FloatNode(*m_impl->imageFile, m_impl->intensityMax));
        scanNode.set("intensityLimits", intensityBox);

        e57::StructureNode timeBox = e57::StructureNode(*m_impl->imageFile);
        timeBox.set("timeStampMinimum", e57::FloatNode(*m_impl->imageFile, m_impl->timeMin));
        timeBox.set("timeStampMaximum", e57::FloatNode(*m_impl->imageFile, m_impl->timeMax));
        scanNode.set("timeStampLimits", timeBox);

        e57::StructureNode colorBox = e57::StructureNode(*m_impl->imageFile);
        colorBox.set("colorRedMinimum", e57::IntegerNode(*m_impl->imageFile, 0));
        colorBox.set("colorRedMaximum", e57::IntegerNode(*m_impl->imageFile, 255));
        colorBox.set("colorGreenMinimum", e57::IntegerNode(*m_impl->imageFile, 0));
        colorBox.set("colorGreenMaximum", e57::IntegerNode(*m_impl->imageFile, 255));
        colorBox.set("colorBlueMinimum", e57::IntegerNode(*m_impl->imageFile, 0));
        colorBox.set("colorBlueMaximum", e57::IntegerNode(*m_impl->imageFile, 255));
        scanNode.set("colorLimits", colorBox);

        m_impl->close();
        
        m_impl->isOpen = false;
    }
    catch (const e57::E57Exception& e) {
        std::string str = e57::Utilities::errorCodeToString(e.errorCode());
        return ;
    }
    catch (...) {
        // 异常处理
    }
}
