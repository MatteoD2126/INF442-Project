import java.io.File;
import java.util.*;

public class Tree {
	private static int N, L;
	private static int[][] edges;
	private static String[] labels;
//	private static int[] edgeWeights;

	private Scanner scanner;
	static String s;

	public int edgeWeight(int u, int v) {
//		add weight to edgeWeights
		if (labels[u].compareTo("?")==0 || labels[v].compareTo("?")==0) {
			System.out.print("\nCan't treat \"?\"");
			return -1;
		}
		if (labels[u].compareTo("$")==0)
			return labels[v].length();
		if (labels[v].compareTo("$")==0)
			return labels[u].length();
		return setDifference(u, v).length();
	}
//	public int edgeWeight(int u, int v) {
//		return edgeWeight(u, v);
//	}

	// TODO
	//	public static int edgeWeights() {
	//	edgeWeights = new int[N-1];
	//	for (int i=0; i<N-1; i++) {
	//		
	//	}
	//	
	//}

	private static String setDifference(int u, int v) {
		// We suppose that the labels are correctly formatted, meaning they are already in alphabetical order ("ABC" is a properly formatted label, "ACB" isn't)

		if (labels[u].compareTo("?")==0 || labels[v].compareTo("?")==0) {
			System.out.print("\nCan't treat \"?\"");
			return "?";
		}

		if (labels[u].compareTo("$")==0)
			return labels[v];

		if (labels[v].compareTo("$")==0)
			return labels[u];
		{
			String setDifference = "";
			int u_length = labels[u].length(), v_length = labels[v].length();
			int i=0, j=0;
			while (i < u_length && j < v_length)	
				if (labels[u].charAt(i) == labels[v].charAt(j)) {
					i++; j++;
				}
				else if (labels[u].charAt(i) < labels[v].charAt(j)) {
					setDifference += labels[u].charAt(i);
					i++;
				}
				else {
					setDifference += labels[v].charAt(j);
					j++;
				}
			if (i < u_length)
				for (; i < u_length; i++)
					setDifference += labels[u].charAt(i);
			else if (j < v_length)
				for (; j < v_length; j++)
					setDifference += labels[v].charAt(j);
			if (setDifference.compareTo("")==0)
				setDifference = "$";
			return setDifference;
		}
	}
//	public static String publicsetDifference(int u, int v) {
//		return setDifference(u, v);
//	}


	private static String intersect(int u, int v) {
		// We suppose that the labels are correctly formatted, meaning they are already in alphabetical order ("ABC" is a properly formatted label, "ACB" isn't)

		if (labels[u].compareTo("?")==0 || labels[v].compareTo("?")==0) {
			System.out.print("\nCan't treat \"?\"");
			return "?";
		}

		if (labels[u].compareTo("$")==0 || labels[v].compareTo("$")==0)
			return "$";

		{
			String intersect = "";
			int u_length = labels[u].length(), v_length = labels[v].length();
			int i=0, j=0;
			while (i < u_length && j < v_length)	
				if (labels[u].charAt(i) == labels[v].charAt(j)) {
					intersect += labels[u].charAt(i);
					i++; j++;
				}
				else if (labels[u].charAt(i) < labels[v].charAt(j))
					i++;
				else
					j++;
			return intersect;
		}
	}
	//	public static String publicIntersect(int u, int v) {
	//	return intersect(u, v);
	//}

	private static String union(int u, int v) {
		// We suppose that the labels are correctly formatted, meaning they are already in alphabetical order ("ABC" is a properly formatted label, "ACB" isn't)

		if (labels[u].compareTo("?")==0 || labels[v].compareTo("?")==0) {
			System.out.print("\nCan't treat \"?\"");
			return "?";
		}

		if (labels[u].compareTo("$")==0)
			return labels[v];

		if (labels[v].compareTo("$")==0)
			return labels[u];

		{
			String union = "";
			int u_length = labels[u].length(), v_length = labels[v].length();
			int i=0, j=0;
			while (i < u_length && j < v_length)	
				if (labels[u].charAt(i) == labels[v].charAt(j)) {
					union += labels[u].charAt(i);
					i++; j++;
				}
				else if (labels[u].charAt(i) < labels[v].charAt(j)) {
					union += labels[u].charAt(i);
					i++;
				}
				else {
					union += labels[v].charAt(j);
					j++;
				}
			if (i < u_length)
				for (; i < u_length; i++)
					union += labels[u].charAt(i);
			else if (j < v_length)
				for (; j < v_length; j++)
					union += labels[v].charAt(j);
			return union;
		}
	}
	//	public static String publicUnion(int u, int v) {
	//	return union(u, v);
	//}


	public void printTree() {
		System.out.printf("N=%d, L=%d", N, L);
		for (int i=0; i<N-1; i++)
			System.out.printf("\n[%d, %d]", edges[i][0], edges[i][1]);
		for (int i=0; i<N; i++) {
			System.out.printf("\nleaf %d: label: %s", i, labels[i]);  // To avoid confusion
			//			System.out.printf("\nleaf %d: label: %s", i+1, labels[i]);
		}
	}

	public void importFile(String file) {
		openFile(file);
		readFile();
		closeFile();
	}

	private void openFile(String file) {
		try {
			this.scanner = new Scanner(new File("C:\\\\Users\\\\dswlm\\\\Documents\\\\Eclipse workspace\\\\Optimal_tree_labelling\\\\src\\\\" + file));
		}
		catch (Exception e) {
			new File(".").getAbsolutePath();
			System.out.println("Error");
		}
	}	

	private void readFile() {
		if (scanner == null)
			System.out.println("scanner is null");
		N = Integer.parseInt(scanner.next());
		L = Integer.parseInt(scanner.next());

		edges = new int[N-1][2];
		for (int i=0; i<N-1; i++) {
			edges[i][0] = Integer.parseInt(scanner.next());
			edges[i][1] = Integer.parseInt(scanner.next());			
		}

		labels = new String[N];
		for (int i=0; i<N; i++) {
			labels[i] = "?";
		}
		int leaf;
		for (int i=0; i<L; i++) {
			leaf = Integer.parseInt(scanner.next());
			labels[leaf-1] = scanner.next();			
		}		
	}

	private void closeFile() {
		scanner.close();
	}

}
