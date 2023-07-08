public class EntropyCluster
{
	/////////////////////////////////////////////
	//  Fuzzy Entropy Clustering Routine
	/////////////////////////////////////////////

	/**
	 * Entropy Clustering calculation
	 * <p>
	 * @param  point input data changed and returned as normalized [0 - 1]
	 * @param  max_dimensions number of dimensions in data
	 * @param  max_points number of points of data
	 * @param  beta minimum required similarity [0 - 1]
	 * @param  gamma fraction of points that must be neighbors otherwise it's an outlier [0 - 1]
	 * @return number of clusters
	 */

	static public int cluster ( data_point[] point, int max_dimensions, int max_points, float beta, float gamma )
	{
		int i, j, hub = 0;
		float entropy[] = new  float[max_points*max_points];  //[MAX_POINTS*MAX_POINTS]
		float entropy_in_cluster;
		float entropy_out_cluster;
		float entropy_cluster[] = new float[max_points];  // [MAX_POINTS]
		float entropy_cluster_min;
		float similarity[] = new float[max_points*max_points];  //[MAX_POINTS*MAX_POINTS]
		float similarity_min;
		float similarity_max;
		float alpha;
		int cluster_nbr;
		float point_min[] = new float[max_dimensions], point_max[] = new float[max_dimensions], point_mean_distance;
		int min_neighbors, neighbors;
		boolean there_are_no_points_to_cluster;

		/////////////////////////////////////////////
		//  Normalize data points
		/////////////////////////////////////////////
		for (j=0; j<max_dimensions; j++)
		{
			point_min[j] = (float) 99e30;
			point_max[j] = (float)-99e30;
			for (i=0; i< max_points; i++ )
			{
				point_min[j] = Math.min(point_min[j], point[i].dimData[j]);
				point_max[j] = Math.max(point_max[j], point[i].dimData[j]);
			}

			for (i=0; i< max_points; i++ )
				if (point_max[j] == point_min[j]) point[i].dimData[j] = .5f;
				else point[i].dimData[j] = (point[i].dimData[j]-point_min[j])/(point_max[j]-point_min[j]);
		}  // data points now [0 - 1] in each dimension


		/////////////////////////////////////////////
		//  Initialize each point's cluster and hub to none (-1)
		//  Compute mean distance between points
		//  Initialize entropy (0.f)
		/////////////////////////////////////////////
		point_mean_distance = 0.f;
		for (i=0; i<max_points; i++)
		{
			point[i].cluster = -1;
			point[i].hub = -1;
			for (j=0; j<max_points; j++)
			{
				entropy[i*max_points+j] = 0.f;
				point_mean_distance += distance(point[i], point[j], max_dimensions);
			}
		}

		if(max_points == 1)	point_mean_distance = 0;
		else point_mean_distance /= (float)(max_points*max_points - max_points);

		/////////////////////////////////////////////
		//  All points the same so one easy cluster
		/////////////////////////////////////////////
		if ( point_mean_distance == 0.f )
		{
			cluster_nbr = 0;
			for (i=0; i< max_points; i++ )
			{
				point[i].cluster = cluster_nbr;
				point[i].hub = max_points/2;
			}
			cluster_nbr++;
			return cluster_nbr;
		}

		/////////////////////////////////////////////
		//  Compute similarity between all points
		/////////////////////////////////////////////
		//  initial computation is unnormalized
		alpha = -(float)Math.log(.5)/point_mean_distance;  // compute alpha
		similarity_min = (float) 99e30;
		similarity_max = (float)-99e30;
		for (i=0; i<max_points; i++)
		{
			for (j=0; j<max_points; j++)
			{
				similarity[max_points*i+j] = (float)Math.exp(-alpha*distance(point[i], point[j], max_dimensions));
				similarity_min = Math.min(similarity_min, similarity[max_points*i+j]);
				similarity_max = Math.max(similarity_max, similarity[max_points*i+j]);
			}
		}

		// normalize similarity [0 - 1]
		for (i=0; i<max_points; i++)
			for (j=0; j<max_points; j++)
				similarity[max_points*i+j] = (similarity[max_points*i+j]-similarity_min)/(similarity_max-similarity_min);

		/////////////////////////////////////////////
		//  Compute entropy between all points
		/////////////////////////////////////////////
		for (i=0; i<max_points; i++)
			for (j=0; j<max_points; j++)
			{
				if (similarity[max_points*i+j] <= 0.f)
					entropy_in_cluster = 0.f;
				else entropy_in_cluster = -similarity[max_points*i+j]*log2(similarity[max_points*i+j]);

				if (similarity[max_points*i+j] >= 1.f)
					entropy_out_cluster = 0.f;
				else  entropy_out_cluster = -(1-similarity[max_points*i+j])*log2(1-similarity[max_points*i+j]);

				entropy[max_points*i+j] = entropy_in_cluster + entropy_out_cluster;
			}

		/////////////////////////////////////////////
		//  Mark outliers so they can't be used as hubs
		//  Marker is -similarity [0 - -1]
		//  Note that -2. below means point has been processed)
		/////////////////////////////////////////////
		min_neighbors = (int)(gamma*max_points);
		for (i=0; i<max_points; i++)
		{
			neighbors = 0;
			for (j=0; j<max_points; j++)
				if ( (i != j) && (similarity[max_points*i+j] >= beta) ) neighbors++;
			if ( neighbors < min_neighbors ) similarity[max_points*i+i] = -similarity[max_points*i+i];
		}

		/////////////////////////////////////////////
		//  Loop to make the clusters
		/////////////////////////////////////////////
		for(cluster_nbr=0;;cluster_nbr++) // go until break because no more data
		{
			there_are_no_points_to_cluster = true; // reset assume no more points points to process
			/////////////////////////////////////////////
			//  Find point with minimum total entropy of surrounding points to become next hub
			/////////////////////////////////////////////
			entropy_cluster_min= (float) Float.MAX_VALUE;
			for (i=0; i<max_points; i++)
			{
				if (similarity[max_points*i+i] < 0.f) continue; // skip used or lonely points as future hubs
				/////////////////////////////////////////////
				//  Find potential cluster entropy; skip already used points (=-2.f)
				/////////////////////////////////////////////
				entropy_cluster[i] = 0.f;
				for (j=0; j<max_points; j++)
					if (similarity[max_points*i+j] > -2.f)
					{
						there_are_no_points_to_cluster = false; // found a point so once more through loop
						entropy_cluster[i] += entropy[max_points*i+j];
					}

				/////////////////////////////////////////////
				//  Find the next hub - point with the minimum entropy
				/////////////////////////////////////////////
				if (entropy_cluster[i] <= entropy_cluster_min)
				{entropy_cluster_min = entropy_cluster[i];
				hub = i;}
			}
			if (there_are_no_points_to_cluster) break; // all done

			/////////////////////////////////////////////
			//  Find members of cluster and mark them
			//
			//  Point at entropy_cluster_min_location 
			//  (hub) and other points within beta of it
			/////////////////////////////////////////////
			for (j=0; j<max_points; j++)
				if ( (j != hub) && (similarity[max_points*hub+j] > -2.f) // non-hub, non-used points
						&& (Math.abs(similarity[max_points*hub+j]) >= beta) ) // neighboring points go in the cluster
				{point[j].cluster = cluster_nbr;
				point[j].hub = hub;
				for (i=0; i<max_points; i++)
				{similarity[max_points*i+j] = -2.f; // mark cluster member's column as used
				similarity[max_points*j+i] = -2.f;} // mark cluster member's row as used
				}

			/////////////////////////////////////////////
			//  Hub is member of cluster so give it the
			//  treatment too now that we are done with it
			/////////////////////////////////////////////
			point[hub].cluster = cluster_nbr;
			point[hub].hub = hub;
			for (i=0; i<max_points; i++)
			{similarity[max_points*i+hub] = -2.f; // mark hub's column as used
			similarity[max_points*hub+i] = -2.f;} // mark hub's row as used
		}
		/////////////////////////////////////////////
		//  End loop to make the clusters
		/////////////////////////////////////////////

		return cluster_nbr;
	}

	/////////////////////////////////////////////
	//  Log base 2
	/////////////////////////////////////////////

	static private float log2(float p)
	{return (float)(Math.log(p)/Math.log(2.f));
	}

	/////////////////////////////////////////////
	//  Distance between 2 points
	/////////////////////////////////////////////
	static private float distance(data_point p1, data_point p2, int max_dim)
	{
		int i;
		float sum = 0;
		for (i=0; i< max_dim; i++)
			sum += (p1.dimData[i] - p2.dimData[i]) * (p1.dimData[i] - p2.dimData[i]);
		return (float)Math.sqrt(sum);
	}

}
