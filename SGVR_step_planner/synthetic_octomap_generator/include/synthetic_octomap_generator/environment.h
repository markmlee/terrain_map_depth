#ifndef SYNTHETIC_DATASET_ENVIRONMENT_H
#define SYNTHETIC_DATASET_ENVIRONMENT_H

#include <iostream>
#include <fstream>
#include <map>

#include <synthetic_octomap_generator/object.h>

/*
 * Geometric information of an environment
 */
class Environment{
public:
    // Constructors
    explicit Environment(const std::string& _filename, const octomap::point3d& _offset = octomap::point3d(0.0f, 0.0f, 0.0f));


    // Get the information of the environment
    unsigned int        size() const { return (unsigned int)objects.size(); }
    octomap::point3d  min_bbx() const;
    octomap::point3d  max_bbx() const;


    // Get the iterator
    std::map<std::string, Object>::const_iterator cbegin() const { return objects.cbegin(); }
    std::map<std::string, Object>::const_iterator cend() const { return objects.cend(); }

protected:
    std::map<std::string, Object> objects;
};

#endif //SYNTHETIC_DATASET_ENVIRONMENT_H
