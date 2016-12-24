#include "shape_detection.h"

int ShapeDetection()
{
    // Points with normals.
    Pwn_vector points;
    // Loads point set from a file. 
    // read_xyz_points_and_normals takes an OutputIterator for storing the points
    // and a property map to store the normal vector with each point.
    std::ifstream stream("data/cube.pwn");
    if (!stream ||
        !CGAL::read_xyz_points_and_normals(stream,
            std::back_inserter(points),
            Point_map(),
            Normal_map()))
    {
        std::cerr << "Error: cannot read file cube.pwn" << std::endl;
        return EXIT_FAILURE;
    }
    // Instantiates shape detection engine.
    Efficient_ransac ransac;
    // Provides the input data.
    ransac.set_input(points);
    // Registers planar shapes via template method.
    ransac.add_shape_factory<Plane>();
    // Detects registered shapes with default parameters.
    ransac.detect();
    // Prints number of detected shapes.
    std::cout << ransac.shapes().end() - ransac.shapes().begin() << " shapes detected." << std::endl;

    return EXIT_SUCCESS;
}

int RetrieveShapes() 
{


    //Efficient_ransac::Shape_range::iterator it = shapes.begin();
    //while (it != shapes.end()) {
    //    boost::shared_ptr<Efficient_ransac::Shape> shape = *it;
    //    // Using Shape_base::info() for printing 
    //    // the parameters of the detected shape.
    //    std::cout << (*it)->info();
    //    // Sums distances of points to detected shapes.
    //    FT sum_distances = 0;
    //    // Iterates through point indices assigned to each detected shape.
    //    std::vector<std::size_t>::const_iterator
    //        index_it = (*it)->indices_of_assigned_points().begin();
    //    while (index_it != (*it)->indices_of_assigned_points().end()) {

    //        // Retrieves point
    //        const Point_with_normal &p = *(points.begin() + (*index_it));
    //        // Adds Euclidean distance between point and shape.
    //        sum_distances += CGAL::sqrt((*it)->squared_distance(p.first));
    //        // Proceeds with next point.
    //        index_it++;
    //    }
    //    // Computes and prints average distance.
    //    FT average_distance = sum_distances / shape->indices_of_assigned_points().size();
    //    std::cout << " average distance: " << average_distance << std::endl;
    //    // Proceeds with next detected shape.
    //    it++;
    //}
    return EXIT_SUCCESS;
}