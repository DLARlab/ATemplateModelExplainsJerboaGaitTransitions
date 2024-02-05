Date: 14 Feb, 2022


Dataset Title: A template model explains jerboa gait transitions across a broad range of speeds (Dataset)


Dataset Creators: J Ding, T.Y. Moore, Z. Gan


Dataset Contact: Jiayu Ding jding14@syr.edu, Talia Moore taliaym@umich.edu


Funding: 
Harvard Chapman Memorial Fellowship to TYM


Key Points:
- We include a SLIP model that can predict the motion of Egyptian jerboa.
- We use a DNN based posture tracking toolkit to extract the trajectories of jerboa motion.
- We use MATLAB to run optimization to match the experimental and simulated trajectories.
- Gait structure of the model was investigated.


Research Overview:
For cursorial animals that maintain high speeds for extended durations of locomotion, transitions between footfall patterns (gaits) predictably occur at distinct speed ranges. How do transitions among gaits occur for non-cursorial animals? Jerboas (Jaculus jaculus) are bipedal hopping rodents that frequently transition between gaits (running, hopping, and skipping) throughout their entire speed range. It has been hypothesized that these non-cursorial bipedal gait transitions are likely to enhance their maneuverability and predator evasion ability. However, it is difficult to use the underlying dynamics of these locomotion patterns to predict gait transitions due to the large number of degrees of freedom expressed by the animals. 

To this end, we used empirical jerboa kinematics and dynamics to develop a unified Spring Loaded Inverted Pendulum model with defined passive swing leg motions. To find periodic solutions of this model, we formulated the gait search as a boundary value problem. We defined a gait as a periodic motion in which all states except for the horizontal position return to their original values after one full stride and discovered a previously undescribed fourth jerboa gait, asymmetrical running. To understand how jerboas change from one gait to another, we employed an optimization approach and used the proposed model to reproduce observed patterns of jerboa gait transitions. During the simulation of a gait transition, the model trajectory depends on the center of mass position and leg angles. The ground reaction forces were modeled as a function of time and a set of model parameters. We solved for the optimal set of model parameters that resulted in dynamics and kinematics that best match the empirical data. We then ran a detailed numerical study of the structure of gait patterns using a continuations approach in which transitions are represented by bifurcations. 
We found two primary mechanisms to increase the range of speeds at which gait transitions can occur. For coupled changes in neutral leg swing angle, the angle can be increased to facilitate gait transitions. The model can also uncouple the left and right neutral leg swing angle to facilitate gait transitions. The majority of the gait transitions involve the skipping gait, both in the experimental observations and in the model. This simulated jerboa is the first dynamic model capable of reproducing biologically relevant gait transitions at a broad range of speeds.


Methodology:
Details of the data collection procedure were reported in a previous publication in which the speeds and acceleration ranges associated with each gait were determined (Moore et al., 2017 Nature Communications). Trials were collected from five captive male jerboas traveling along a narrow track (2×0.15×0.4 m^3) over a two-axis force platform (0.06×0.12m^2) and past a high-speed video camera recording at 500 fps. We visually categorized the gait of each stride by footfall pattern.

Both feet striking and lifting off simultaneously were considered hopping. Overlapping but non-simultaneous foot strikes were considered skipping, according to previous work (Moore et al., 2017 Nature Communications). If the same leg maintained the leading foot position, this gait would be equivalent to a bipedal gallop, as defined in previous gait research (Schropfer et al., 1985 Mammalia; Gan et al., 2018 Journal of the Royal Society Interface). An aerial phase between each foot strike was considered running if each aerial phase was approximately the same duration.

To extract the kinematic data (i.e., center of mass (COM) locations and leg angles over one stride) from the video recordings, we used DeepLabCut, a markerless pose estimation framework leveraging a deep neutral network (DNN). In this study, 35 videos that contained a whole stride of a single gait pattern were used to train the DNN.

All three common jerboa gaits reported in (Moore et al., 2017 Nature Communications) (i.e., hopping, skipping, running) were included in this study. Roughly 1/3 of the total frames of each video were selected as the training data set.  In these frames, we manually labelled the location of the eye, the tail-base, and the two feet. We estimated the COM location as the midpoint between the eye and the tail-base. Then the leg angles were calculated as the orientation of the line segments connecting the COM to the feet.

The proposed model used in this study consists of a point mass as the main body, with mass m, and two massless legs. The vertical and horizontal positions of the main body were defined by the variables x(t) and y(t), respectively. Left and right legs were modeled as massless linear springs with resting leg length l_o and total spring stiffness k. Both legs were connected to the main body through frictionless rotational joints, with the joint angle alpha_i (t) measured from the vertical axis (positive in the counterclockwise direction). 

In contrast to the convectional SLIP model, which ignores swing leg motions by setting the leg to predefined angles of attack immediately after lifting off, we added a torsional passive spring to control the leg swing motion during the flight phase of each leg. The torsional spring directly connected the leg to the main body at angle, phi_i (hereafter referred to as the neutral swing leg angle), measured with respect to the vertical direction.

By fixing the oscillation frequency omega, this torsional spring dictates the swing leg rotational speed and amplitude and determines the desired contact angle at the moment of touchdown. Because we can assume that the torsional spring stiffness and the foot mass have infinitesimal values, they do not affect stance leg kinematics or dynamics (Gan et al., 2018 Journal of the Royal Society Interface). In our work, we ran optimizations to fit the trajectories of leg angles to determine the oscillation frequency \omega. 

Moore, T.Y., Cooper, K.L., Biewener. A.A. et al. Unpredictability of escape trajectory explains predator evasion ability and microhabitat preference of desert rodents. Nat Commun 8, 440 (2017). https://doi.org/10.1038/s414467-017-00373-2

Schroffer, R., Klenner-Fringes, B. & Naumer, E. (1985). Locomotion pattern and habit utilization of the two jerboas Jaculus jaculus and Jaculus orientalis (Rodentia, Dipodidae), 49(4), 445-454. Https://doi.org/10.1515/mamml.1985.49.4.445

Gan Z, Yesilevskiy Y, Zaytsev P, Remy CD. 2018 All common bipedal gaits emerge from a single passive model. J. R. Soc. Interface 15: 20180455. http://dx.doi.org/10.1098/rsif.2018.0455


Instrument and/or Software specifications:
We used DeepLabCut (Mathis Laboratory|EPFL, Genève, Switzerland) toolkit to extract the trajectories of jerboas. We used MATLAB (2021a, Mathworks, Natick, MA) to process the data, implement the model, and to build our optimization and numerical algorithms.


Files contained:
The complement resources are organized as following:
In the project folder, the script ‘Main.m’ is used to call all functions and data files to reproduce the solutions, figures, and animations.
All custom functions used in this project are stored in the folder called ‘Stored_Functions’. Jerboa data extracted from the experiments are stored the rest of the folders for individual sections.


Instructions:
1. Download the zipped project folder and unzip all files.
2. Open the ‘Main.m’ file in Matlab and change the file location to the current working folder.
3. Execute the file by clicking on the ‘Run button and each section will be executed automatically. Alternatively, one can click on the ‘Run Section’ button and only show the results of one section.
4. In section 1 of the ‘Main.m’ file, an initialization step is required to initialize the environment and add all functions and data files to the Matlab searchable path.
5. In section 2 of the ‘Main.m’ file, the trajectories and animations of 5 different gaits are plotted. And the Figure 4 in the manuscript is reproduced.
6. In section 3 of the ‘Main.m’ file, an optimization example is demonstrated, and the Figure 2 is replotted.
7. In section 4 of the ‘Main.m’ file, a demo of continuation problem (how we found the solution branches) is shown and the Figure 8 in the manuscript is replotted.
 



Related publication(s):
Ding, J, Moore, TY, Gan, Z. (2021) A template model explains jerboa gait transitions across a broad range of speeds


Use and Access:
Attribution - NonCommercial 4.0 International (CC BY-NC 4.0)


To Cite Data:
Ding, J., Moore, T., Gan, Z. A template model explains jerboa gait transitions across a broad range of speeds [Data set], University of Michigan - Deep Blue Data. https://doi.org/10.7302/ewaa-qm16
