# Drone-Delivery-Landing-Zone-Detection
Algorithms and benchmarking strategies.

Implemented algorithms to date and included in the Monte-carlo based benchmarking strategy. 

1. Traditional rule based mathematical models with preset heuristic rules: 
    1.1 Region_Growing_Segmentation
    1.2 seq_overlap
    1.3 kdtree_InflatingCircles
    1.4 sequentialApproachKdtree
    1.5 sequentialApproach


![Region_Growing_Segmentation](media/regiongrowingseg.drawio.png)

![seq_overlap](media/seq_throry_imple_resul.drawio.png)

![kdtree_InflatingCircles](media/inflating_circleskdtree.drawio.png)

Note: For detailed explaination of implementation with flow charts and pseudo code refere to the paper published here (published by end of aug 2025).



We use .yaml file to configure algorithm selection and their parameters.
You will find the self explainatory file at below path,
/lib/config/monte_carlo_benchmarking_config.yaml

To test functionality of individual algorithm on own .pcd file and vizualize its output use algo_testing_own_pcd_config.yaml


Steps to run the repo - montecarlo based benchmarking:
- Code related to montecarlo based benchmarking yet to be updated  