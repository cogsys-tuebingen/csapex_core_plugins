#ifndef POLAR_CLUSTERING_HPP
#define POLAR_CLUSTERING_HPP

/// SYSTEM
#include <pcl/pcl_base.h>
#include <pcl/search/pcl_search.h>


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Decompose a region of space into clusters based on the Euclidean distance between points
  * \param cloud the point cloud message
  * \param tree the spatial locator (e.g., kd-tree) used for nearest neighbors searching
  * \note the tree has to be created as a spatial locator on \a cloud
  * \param tolerance the spatial cluster tolerance as a measure in L2 Euclidean space
  * \param clusters the resultant clusters containing point indices (as a vector of PointIndices)
  * \param min_pts_per_cluster minimum number of points that a cluster may contain (default: 1)
  * \param max_pts_per_cluster maximum number of points that a cluster may contain (default: max int)
  * \ingroup segmentation
  */
template <typename PointT> void
extractPolarEuclideanClusters (
        const pcl::PointCloud<PointT> &cloud, const boost::shared_ptr<pcl::search::Search<PointT> > &tree,
        float alpha,
        float tolerance, std::vector<pcl::PointIndices> &clusters,
        unsigned int min_pts_per_cluster = 1, unsigned int max_pts_per_cluster = (std::numeric_limits<int>::max) ());

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Decompose a region of space into clusters based on the Euclidean distance between points
  * \param cloud the point cloud message
  * \param indices a list of point indices to use from \a cloud
  * \param tree the spatial locator (e.g., kd-tree) used for nearest neighbors searching
  * \note the tree has to be created as a spatial locator on \a cloud and \a indices
  * \param tolerance the spatial cluster tolerance as a measure in L2 Euclidean space
  * \param clusters the resultant clusters containing point indices (as a vector of PointIndices)
  * \param min_pts_per_cluster minimum number of points that a cluster may contain (default: 1)
  * \param max_pts_per_cluster maximum number of points that a cluster may contain (default: max int)
  * \ingroup segmentation
  */
template <typename PointT> void
extractPolarEuclideanClusters (
        const pcl::PointCloud<PointT> &cloud, const std::vector<int> &indices,
        const boost::shared_ptr<pcl::search::Search<PointT> > &tree, float alpha,
        float tolerance,
        std::vector<pcl::PointIndices> &clusters,
        unsigned int min_pts_per_cluster = 1, unsigned int max_pts_per_cluster = (std::numeric_limits<int>::max) ());

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Decompose a region of space into clusters based on the euclidean distance between points, and the normal
  * angular deviation
  * \param cloud the point cloud message
  * \param normals the point cloud message containing normal information
  * \param tree the spatial locator (e.g., kd-tree) used for nearest neighbors searching
  * \note the tree has to be created as a spatial locator on \a cloud
  * \param tolerance the spatial cluster tolerance as a measure in the L2 Euclidean space
  * \param clusters the resultant clusters containing point indices (as a vector of PointIndices)
  * \param eps_angle the maximum allowed difference between normals in radians for cluster/region growing
  * \param min_pts_per_cluster minimum number of points that a cluster may contain (default: 1)
  * \param max_pts_per_cluster maximum number of points that a cluster may contain (default: max int)
  * \ingroup segmentation
  */
template <typename PointT, typename Normal> void
extractPolarEuclideanClusters (
        const pcl::PointCloud<PointT> &cloud, const pcl::PointCloud<Normal> &normals,
        float alpha,
        float tolerance, const boost::shared_ptr<pcl::KdTree<PointT> > &tree,
        std::vector<pcl::PointIndices> &clusters, double eps_angle,
        unsigned int min_pts_per_cluster = 1,
        unsigned int max_pts_per_cluster = (std::numeric_limits<int>::max) ())
{
    if (tree->getInputCloud ()->points.size () != cloud.points.size ())
    {
        PCL_ERROR ("[pcl::extractPolarEuclideanClusters] Tree built for a different point cloud dataset (%zu) than the input cloud (%zu)!\n", tree->getInputCloud ()->points.size (), cloud.points.size ());
        return;
    }
    if (cloud.points.size () != normals.points.size ())
    {
        PCL_ERROR ("[pcl::extractPolarEuclideanClusters] Number of points in the input point cloud (%zu) different than normals (%zu)!\n", cloud.points.size (), normals.points.size ());
        return;
    }

    // Create a bool vector of processed point indices, and initialize it to false
    std::vector<bool> processed (cloud.points.size (), false);

    std::vector<int> nn_indices;
    std::vector<float> nn_distances;
    // Process all points in the indices vector
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
        if (processed[i])
            continue;

        std::vector<unsigned int> seed_queue;
        int sq_idx = 0;
        seed_queue.push_back (static_cast<int> (i));

        processed[i] = true;

        while (sq_idx < static_cast<int> (seed_queue.size ()))
        {
            // Search for sq_idx
            const PointT& pt = cloud.at(sq_idx);
            auto r = std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
            float scaled_tolerance = tolerance * std::pow(r, alpha);
            if (!tree->radiusSearch (seed_queue[sq_idx], scaled_tolerance, nn_indices, nn_distances))
            {
                sq_idx++;
                continue;
            }

            for (size_t j = 1; j < nn_indices.size (); ++j)             // nn_indices[0] should be sq_idx
            {
                if (processed[nn_indices[j]])                         // Has this point been processed before ?
                    continue;

                //processed[nn_indices[j]] = true;
                // [-1;1]
                double dot_p = normals.points[i].normal[0] * normals.points[nn_indices[j]].normal[0] +
                        normals.points[i].normal[1] * normals.points[nn_indices[j]].normal[1] +
                        normals.points[i].normal[2] * normals.points[nn_indices[j]].normal[2];
                if ( fabs (acos (dot_p)) < eps_angle )
                {
                    processed[nn_indices[j]] = true;
                    seed_queue.push_back (nn_indices[j]);
                }
            }

            sq_idx++;
        }

        // If this queue is satisfactory, add to the clusters
        if (seed_queue.size () >= min_pts_per_cluster && seed_queue.size () <= max_pts_per_cluster)
        {
            pcl::PointIndices r;
            r.indices.resize (seed_queue.size ());
            for (size_t j = 0; j < seed_queue.size (); ++j)
                r.indices[j] = seed_queue[j];

            // These two lines should not be needed: (can anyone confirm?) -FF
            std::sort (r.indices.begin (), r.indices.end ());
            r.indices.erase (std::unique (r.indices.begin (), r.indices.end ()), r.indices.end ());

            r.header = cloud.header;
            clusters.push_back (r);   // We could avoid a copy by working directly in the vector
        }
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Decompose a region of space into clusters based on the euclidean distance between points, and the normal
  * angular deviation
  * \param cloud the point cloud message
  * \param normals the point cloud message containing normal information
  * \param indices a list of point indices to use from \a cloud
  * \param tree the spatial locator (e.g., kd-tree) used for nearest neighbors searching
  * \note the tree has to be created as a spatial locator on \a cloud
  * \param tolerance the spatial cluster tolerance as a measure in the L2 Euclidean space
  * \param clusters the resultant clusters containing point indices (as PointIndices)
  * \param eps_angle the maximum allowed difference between normals in degrees for cluster/region growing
  * \param min_pts_per_cluster minimum number of points that a cluster may contain (default: 1)
  * \param max_pts_per_cluster maximum number of points that a cluster may contain (default: max int)
  * \ingroup segmentation
  */
template <typename PointT, typename Normal>
void extractPolarEuclideanClusters (
        const pcl::PointCloud<PointT> &cloud, const pcl::PointCloud<Normal> &normals,
        const std::vector<int> &indices, const boost::shared_ptr<pcl::KdTree<PointT> > &tree,
        float alpha,
        float tolerance, std::vector<pcl::PointIndices> &clusters, double eps_angle,
        unsigned int min_pts_per_cluster = 1,
        unsigned int max_pts_per_cluster = (std::numeric_limits<int>::max) ())
{
    // \note If the tree was created over <cloud, indices>, we guarantee a 1-1 mapping between what the tree returns
    //and indices[i]
    if (tree->getInputCloud ()->points.size () != cloud.points.size ())
    {
        PCL_ERROR ("[pcl::extractPolarEuclideanClusters] Tree built for a different point cloud dataset (%zu) than the input cloud (%zu)!\n", tree->getInputCloud ()->points.size (), cloud.points.size ());
        return;
    }
    if (tree->getIndices ()->size () != indices.size ())
    {
        PCL_ERROR ("[pcl::extractPolarEuclideanClusters] Tree built for a different set of indices (%zu) than the input set (%zu)!\n", tree->getIndices ()->size (), indices.size ());
        return;
    }
    if (cloud.points.size () != normals.points.size ())
    {
        PCL_ERROR ("[pcl::extractPolarEuclideanClusters] Number of points in the input point cloud (%zu) different than normals (%zu)!\n", cloud.points.size (), normals.points.size ());
        return;
    }
    // Create a bool vector of processed point indices, and initialize it to false
    std::vector<bool> processed (cloud.points.size (), false);

    std::vector<int> nn_indices;
    std::vector<float> nn_distances;
    // Process all points in the indices vector
    for (size_t i = 0; i < indices.size (); ++i)
    {
        if (processed[indices[i]])
            continue;

        std::vector<int> seed_queue;
        int sq_idx = 0;
        seed_queue.push_back (indices[i]);

        processed[indices[i]] = true;

        while (sq_idx < static_cast<int> (seed_queue.size ()))
        {
            // Search for sq_idx
            const PointT& pt = cloud.at(sq_idx);
            auto r = std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
            float scaled_tolerance = tolerance * std::pow(r, alpha);
            if (!tree->radiusSearch (cloud.points[seed_queue[sq_idx]], scaled_tolerance, nn_indices, nn_distances))
            {
                sq_idx++;
                continue;
            }

            for (size_t j = 1; j < nn_indices.size (); ++j)             // nn_indices[0] should be sq_idx
            {
                if (processed[nn_indices[j]])                             // Has this point been processed before ?
                    continue;

                //processed[nn_indices[j]] = true;
                // [-1;1]
                double dot_p =
                        normals.points[indices[i]].normal[0] * normals.points[indices[nn_indices[j]]].normal[0] +
                        normals.points[indices[i]].normal[1] * normals.points[indices[nn_indices[j]]].normal[1] +
                        normals.points[indices[i]].normal[2] * normals.points[indices[nn_indices[j]]].normal[2];
                if ( fabs (acos (dot_p)) < eps_angle )
                {
                    processed[nn_indices[j]] = true;
                    seed_queue.push_back (nn_indices[j]);
                }
            }

            sq_idx++;
        }

        // If this queue is satisfactory, add to the clusters
        if (seed_queue.size () >= min_pts_per_cluster && seed_queue.size () <= max_pts_per_cluster)
        {
            pcl::PointIndices r;
            r.indices.resize (seed_queue.size ());
            for (size_t j = 0; j < seed_queue.size (); ++j)
                r.indices[j] = seed_queue[j];

            // These two lines should not be needed: (can anyone confirm?) -FF
            std::sort (r.indices.begin (), r.indices.end ());
            r.indices.erase (std::unique (r.indices.begin (), r.indices.end ()), r.indices.end ());

            r.header = cloud.header;
            clusters.push_back (r);
        }
    }
}

/** \brief Sort clusters method (for std::sort).
    * \ingroup segmentation
    */
inline bool
comparePointClusters (const pcl::PointIndices &a, const pcl::PointIndices &b)
{
    return (a.indices.size () < b.indices.size ());
}

template <typename PointT>
class PolarClustering : public pcl::PCLBase<PointT>
{
    typedef pcl::PCLBase<PointT> BasePCLBase;

public:
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    typedef typename pcl::search::Search<PointT> KdTree;
    typedef typename pcl::search::Search<PointT>::Ptr KdTreePtr;

    typedef pcl::PointIndices::Ptr PointIndicesPtr;
    typedef pcl::PointIndices::ConstPtr PointIndicesConstPtr;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Empty constructor. */
    PolarClustering () : tree_ (),
        cluster_tolerance_ (0),
        opening_angle_(1.0),
        min_pts_per_cluster_ (1),
        max_pts_per_cluster_ (std::numeric_limits<int>::max ())
    {}

    /** \brief Provide a pointer to the search object.
        * \param[in] tree a pointer to the spatial search object.
        */
    inline void
    setSearchMethod (const KdTreePtr &tree)
    {
        tree_ = tree;
    }

    /** \brief Get a pointer to the search method used.
       *  @todo fix this for a generic search tree
       */
    inline KdTreePtr
    getSearchMethod () const
    {
        return (tree_);
    }

    /** \brief Set the spatial cluster tolerance as a measure in the L2 Euclidean space
        * \param[in] tolerance the spatial cluster tolerance as a measure in the L2 Euclidean space
        */
    inline void
    setClusterTolerance (double tolerance)
    {
        cluster_tolerance_ = tolerance;
    }

    /** \brief Get the spatial cluster tolerance as a measure in the L2 Euclidean space. */
    inline double
    getClusterTolerance () const
    {
        return (cluster_tolerance_);
    }

    inline void
    setOpeningAngle (double opening_angle)
    {
        opening_angle_ = opening_angle;
    }

    inline double
    getOpeningAngle () const
    {
        return (opening_angle_);
    }

    /** \brief Set the minimum number of points that a cluster needs to contain in order to be considered valid.
        * \param[in] min_cluster_size the minimum cluster size
        */
    inline void
    setMinClusterSize (int min_cluster_size)
    {
        min_pts_per_cluster_ = min_cluster_size;
    }

    /** \brief Get the minimum number of points that a cluster needs to contain in order to be considered valid. */
    inline int
    getMinClusterSize () const
    {
        return (min_pts_per_cluster_);
    }

    /** \brief Set the maximum number of points that a cluster needs to contain in order to be considered valid.
        * \param[in] max_cluster_size the maximum cluster size
        */
    inline void
    setMaxClusterSize (int max_cluster_size)
    {
        max_pts_per_cluster_ = max_cluster_size;
    }

    /** \brief Get the maximum number of points that a cluster needs to contain in order to be considered valid. */
    inline int
    getMaxClusterSize () const
    {
        return (max_pts_per_cluster_);
    }

    /** \brief Cluster extraction in a PointCloud given by <setInputCloud (), setIndices ()>
        * \param[out] clusters the resultant point clusters
        */
    void
    extract (std::vector<pcl::PointIndices> &clusters)
    {
        if (!initCompute () ||
                (input_ != 0   && input_->points.empty ()) ||
                (indices_ != 0 && indices_->empty ()))
        {
            clusters.clear ();
            return;
        }

        // Initialize the spatial locator
        if (!tree_)
        {
            if (input_->isOrganized ())
                tree_.reset (new pcl::search::OrganizedNeighbor<PointT> ());
            else
                tree_.reset (new pcl::search::KdTree<PointT> (false));
        }

        // Send the input dataset to the spatial locator
        tree_->setInputCloud (input_, indices_);
        extractPolarEuclideanClusters (*input_, *indices_, tree_,
                                       static_cast<float>(opening_angle_), static_cast<float> (cluster_tolerance_),
                                       clusters, min_pts_per_cluster_, max_pts_per_cluster_);

        //tree_->setInputCloud (input_);
        //extractPolarEuclideanClusters (*input_, tree_, cluster_tolerance_, clusters, min_pts_per_cluster_, max_pts_per_cluster_);

        // Sort the clusters based on their size (largest one first)
        std::sort (clusters.rbegin (), clusters.rend (), comparePointClusters);

        deinitCompute ();
    }

protected:
    // Members derived from the base class
    using BasePCLBase::input_;
    using BasePCLBase::indices_;
    using BasePCLBase::initCompute;
    using BasePCLBase::deinitCompute;

    /** \brief A pointer to the spatial search object. */
    KdTreePtr tree_;

    /** \brief The spatial cluster tolerance as a measure in the L2 Euclidean space. */
    double cluster_tolerance_;

    double opening_angle_;

    /** \brief The minimum number of points that a cluster needs to contain in order to be considered valid (default = 1). */
    int min_pts_per_cluster_;

    /** \brief The maximum number of points that a cluster needs to contain in order to be considered valid (default = MAXINT). */
    int max_pts_per_cluster_;

    /** \brief Class getName method. */
    virtual std::string getClassName () const { return ("EuclideanClusterExtraction"); }

};

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
extractPolarEuclideanClusters (const pcl::PointCloud<PointT> &cloud,
                               const boost::shared_ptr<pcl::search::Search<PointT> > &tree,
                               float alpha,
                               float tolerance, std::vector<pcl::PointIndices> &clusters,
                               unsigned int min_pts_per_cluster,
                               unsigned int max_pts_per_cluster)
{
    if (tree->getInputCloud ()->points.size () != cloud.points.size ())
    {
        PCL_ERROR ("[pcl::extractPolarEuclideanClusters] Tree built for a different point cloud dataset (%zu) than the input cloud (%zu)!\n", tree->getInputCloud ()->points.size (), cloud.points.size ());
        return;
    }
    // Check if the tree is sorted -- if it is we don't need to check the first element
    int nn_start_idx = tree->getSortedResults () ? 1 : 0;
    // Create a bool vector of processed point indices, and initialize it to false
    std::vector<bool> processed (cloud.points.size (), false);

    std::vector<int> nn_indices;
    std::vector<float> nn_distances;
    // Process all points in the indices vector
    for (int i = 0; i < static_cast<int> (cloud.points.size ()); ++i)
    {
        if (processed[i])
            continue;

        std::vector<int> seed_queue;
        int sq_idx = 0;
        seed_queue.push_back (i);

        processed[i] = true;

        while (sq_idx < static_cast<int> (seed_queue.size ()))
        {
            // Search for sq_idx
            const PointT& pt = cloud.at(sq_idx);
            auto r = std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
            float scaled_tolerance = tolerance * std::pow(r, alpha);
            if (!tree->radiusSearch (seed_queue[sq_idx], scaled_tolerance, nn_indices, nn_distances))
            {
                sq_idx++;
                continue;
            }

            for (size_t j = nn_start_idx; j < nn_indices.size (); ++j)             // can't assume sorted (default isn't!)
            {
                if (nn_indices[j] == -1 || processed[nn_indices[j]])        // Has this point been processed before ?
                    continue;

                // Perform a simple Euclidean clustering
                seed_queue.push_back (nn_indices[j]);
                processed[nn_indices[j]] = true;
            }

            sq_idx++;
        }

        // If this queue is satisfactory, add to the clusters
        if (seed_queue.size () >= min_pts_per_cluster && seed_queue.size () <= max_pts_per_cluster)
        {
            pcl::PointIndices r;
            r.indices.resize (seed_queue.size ());
            for (size_t j = 0; j < seed_queue.size (); ++j)
                r.indices[j] = seed_queue[j];

            // These two lines should not be needed: (can anyone confirm?) -FF
            std::sort (r.indices.begin (), r.indices.end ());
            r.indices.erase (std::unique (r.indices.begin (), r.indices.end ()), r.indices.end ());

            r.header = cloud.header;
            clusters.push_back (r);   // We could avoid a copy by working directly in the vector
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////
/** @todo: fix the return value, make sure the exit is not needed anymore*/
template <typename PointT> void
extractPolarEuclideanClusters (const pcl::PointCloud<PointT> &cloud,
                               const std::vector<int> &indices,
                               const boost::shared_ptr<pcl::search::Search<PointT> > &tree,
                               float alpha,
                               float tolerance, std::vector<pcl::PointIndices> &clusters,
                               unsigned int min_pts_per_cluster,
                               unsigned int max_pts_per_cluster)
{
    // \note If the tree was created over <cloud, indices>, we guarantee a 1-1 mapping between what the tree returns
    //and indices[i]
    if (tree->getInputCloud ()->points.size () != cloud.points.size ())
    {
        PCL_ERROR ("[pcl::extractPolarEuclideanClusters] Tree built for a different point cloud dataset (%zu) than the input cloud (%zu)!\n", tree->getInputCloud ()->points.size (), cloud.points.size ());
        return;
    }
    if (tree->getIndices ()->size () != indices.size ())
    {
        PCL_ERROR ("[pcl::extractPolarEuclideanClusters] Tree built for a different set of indices (%zu) than the input set (%zu)!\n", tree->getIndices ()->size (), indices.size ());
        return;
    }
    // Check if the tree is sorted -- if it is we don't need to check the first element
    int nn_start_idx = tree->getSortedResults () ? 1 : 0;

    // Create a bool vector of processed point indices, and initialize it to false
    std::vector<bool> processed (cloud.points.size (), false);

    std::vector<int> nn_indices;
    std::vector<float> nn_distances;
    // Process all points in the indices vector
    for (int i = 0; i < static_cast<int> (indices.size ()); ++i)
    {
        if (processed[indices[i]])
            continue;

        std::vector<int> seed_queue;
        int sq_idx = 0;
        seed_queue.push_back (indices[i]);

        processed[indices[i]] = true;

        while (sq_idx < static_cast<int> (seed_queue.size ()))
        {
            // Search for sq_idx
            const PointT& pt = cloud.at(sq_idx);
            auto r = std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
            float scaled_tolerance = tolerance * std::pow(r, alpha);
            int ret = tree->radiusSearch (cloud.points[seed_queue[sq_idx]], scaled_tolerance, nn_indices, nn_distances);
            if( ret == -1)
            {
                PCL_ERROR("[pcl::extractPolarEuclideanClusters] Received error code -1 from radiusSearch\n");
                exit(0);
            }
            if (!ret)
            {
                sq_idx++;
                continue;
            }

            for (size_t j = nn_start_idx; j < nn_indices.size (); ++j)             // can't assume sorted (default isn't!)
            {
                if (nn_indices[j] == -1 || processed[nn_indices[j]])        // Has this point been processed before ?
                    continue;

                // Perform a simple Euclidean clustering
                seed_queue.push_back (nn_indices[j]);
                processed[nn_indices[j]] = true;
            }

            sq_idx++;
        }

        // If this queue is satisfactory, add to the clusters
        if (seed_queue.size () >= min_pts_per_cluster && seed_queue.size () <= max_pts_per_cluster)
        {
            pcl::PointIndices r;
            r.indices.resize (seed_queue.size ());
            for (size_t j = 0; j < seed_queue.size (); ++j)
                // This is the only place where indices come into play
                r.indices[j] = seed_queue[j];

            // These two lines should not be needed: (can anyone confirm?) -FF
            //r.indices.assign(seed_queue.begin(), seed_queue.end());
            std::sort (r.indices.begin (), r.indices.end ());
            r.indices.erase (std::unique (r.indices.begin (), r.indices.end ()), r.indices.end ());

            r.header = cloud.header;
            clusters.push_back (r);   // We could avoid a copy by working directly in the vector
        }
    }
}


#endif // POLAR_CLUSTERING_HPP
