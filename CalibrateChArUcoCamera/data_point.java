public class data_point implements Comparable<data_point>
 // okay if we always want to sort only one way for example, ascending hub and image Y
 // otherwise should use Comparator to define multiple different classes of sorting sequences
{
	public int cluster;
	public int hub;
	public float[] dimData;

	public data_point(int max_dimensions)
	{
		dimData = new float[max_dimensions];
	
	} 
	
	public String toString()
	{
		return
		String.format("Cluster %d, hub %d", cluster, hub);
	}

	// @Override
    // public boolean equals(data_point other) {
	// 	//return other.getName().equals(this.getName());
	// 	return
	// 		this.hub        == other.hub
	// 		 &&
	// 		this.dimData[1] == other.dimData[1];
    // }
 
    @Override
    public int compareTo(data_point other) {
		// return other - this  descending
		// return this - other  ascending
        
		if (this.hub > other.hub) return +1;
		if (this.hub < other.hub) return -1;
		if (this.dimData[1] > other.dimData[1]) return +1;
		if (this.dimData[1] < other.dimData[1]) return -1;
		return 0;
    }
}