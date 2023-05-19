void processInputFile(const std::string &filename, Graph &graph)
{
    std::ifstream is(filename);
    std::string line;
    
    

    if (FILETYPE == 0)
    {
        int nb_points, nb_arcs;
        std::getline(is, line, '\n');
        std::stringstream ls;
        ls << line;
        ls >> nb_points >> nb_arcs;
        int v1, v2;
        double weight;
        int res;
        int v1, v2;
        double weight;
        int res;
        for (int j = 0; j < nb_arcs; j++)
        {
            std::getline(is, line, '\n');
            ls.str(line);
            ls.clear();
            ls >> v1 >> v2 >> weight >> res;
            graph.addArc(v1, v2, weight);
        }
    }
    else {
        int nb_points, nb_arcs, nb_res;
        std::getline(is, line, '\n');
        std::stringstream ls;
        ls << line;
        ls >> nb_points >> nb_arcs >> nb_res;

        // skip some lines
        // TODO
        // skip the lower limit on the resources consumed on the chosen path
        for (int i = 0; i < nb_res; i++){
            std::getline(is, line, '\n');
        }
        // skip the upper limit on the resources consumed on the chosen path
        for (int i = 0; i < nb_res; i++){
            std::getline(is, line, '\n');
        }
        // skip the amount of each resource k (k=1,...,K) consumed in passing through each vertex
        for (int i = 0; i < nb_points * nb_res; i++){
            std::getline(is, line, '\n');
        }
        
        int v1, v2, res;
        double weight;
        for (int j = 0; j < nb_arcs; j++)
        {
            std::getline(is, line, '\n');
            ls.str(line);
            ls.clear();
            ls >> v1 >> v2 >> weight >> res;
            graph.addArc(v1, v2, weight);
        }
    }
}
