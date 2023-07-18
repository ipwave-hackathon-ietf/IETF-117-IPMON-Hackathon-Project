# IETF-117-IPMON-Hackathon-Project
This is a simulation of a 5G drone network.

To run this project follow these steps:

(1) After a successful installation of OMNeT++ 5.6.2, Download and import the INET 4.2.5 into your omnet++ workspace.
    - You can get the inet from https://inet.omnetpp.org/Download.html
    - Buld to the project by Right clicking the inet project and cclicking the Build Project or Pressing the Ctrl + B
    
(2) Download the IETF-117-IPMON-Hackathon Project
    - It contains two projects: SIMU5G which is the 5G simulation and CANA-IETF-117 Which is a flying ad hoc network Simulation.
    (a) Fistly, import the SIMU5G into your workspace. Right click the project, choose Project References, tick the inet, and click apply and close.
        Build the project by Right clicking the inet project and cclicking the Build Project or Pressing the Ctrl + B.
    (b) Secondary, impoort the CANA-IETF-117  into your work space. ight click the project, choose Project References, tick the inet and simu5G, and click apply and close.
        Build the project by Right clicking the inet project and cclicking the Build Project or Pressing the Ctrl + B.

        Now your project is ready!
        
(3) Run the Hackathon project by entering CANA-IETF-117 > simu5gdrone > right click the omnetpp.ini > Run as > OMNeT++ Simulation.

Enjoy our simulation!

