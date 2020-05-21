#include <synthetic_octomap_generator/environment.h>

Environment::Environment(const std::string& _filename, const octomap::point3d& _offset)
{
    // Open the file [.txt]
    std::ifstream environment_file(_filename);
    if(!environment_file.is_open()){
        std::cout << "WARN: " << _filename << " is not opened." << std::endl;
        return;
    }

    // File format: "name" "center X" "center Y" "center Z" "length X" "length Y" "length Z"
    // We assume all the objects are axis-aligned, i.e., box
    while(!environment_file.eof()){
        std::string name;
        octomap::point3d center;
        octomap::point3d length;

        environment_file >> name;
        environment_file >> center.x() >> center.y() >> center.z();
        environment_file >> length.x() >> length.y() >> length.z();
        center += _offset;

        if(!name.empty())
            objects[name].set_bbx(center, length);
    }

    // Close the file
    environment_file.close();
}

octomap::point3d Environment::min_bbx() const
{
    float max_value = std::numeric_limits<float>::max();
    octomap::point3d min_bbx(max_value, max_value, max_value);

    for(auto it = objects.cbegin(); it != objects.cend(); ++it){
        octomap::point3d min_vertex = it->second.get_min_bbx();
        if(min_vertex.x() < min_bbx.x())    min_bbx.x() = min_vertex.x();
        if(min_vertex.y() < min_bbx.y())    min_bbx.y() = min_vertex.y();
        if(min_vertex.z() < min_bbx.z())    min_bbx.z() = min_vertex.z();
    }

    return min_bbx;
}

octomap::point3d Environment::max_bbx() const
{
    float max_value = std::numeric_limits<float>::max();
    octomap::point3d max_bbx(-max_value, -max_value, -max_value);

    for(auto it = objects.cbegin(); it != objects.cend(); ++it){
        octomap::point3d max_vertex = it->second.get_max_bbx();
        if(max_vertex.x() > max_bbx.x())    max_bbx.x() = max_vertex.x();
        if(max_vertex.y() > max_bbx.y())    max_bbx.y() = max_vertex.y();
        if(max_vertex.z() > max_bbx.z())    max_bbx.z() = max_vertex.z();
    }

    return max_bbx;
}