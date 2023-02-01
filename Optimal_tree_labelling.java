import java.io.File;
import java.util.Scanner;

public class Optimal_tree_labelling {

	public static void main(String[] args) {
		Tree tree = new Tree();
		tree.importFile("0.in");
		tree.printTree();
		int u = 98;
		int v = 99;
		System.out.printf("\nEdge [%d, %d] weight: %d", u,v, tree.edgeWeight(u,v));
//		System.out.printf("\nEdge [%d, %d] weight: %d", u,v, tree.union(u,v));
	}

}
