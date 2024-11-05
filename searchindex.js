Search.setIndex({"docnames": ["cpp_src/packet_handler_mech", "cpp_src/packet_raw", "cpp_src/reciever_socket", "cpp_src/ros_driver", "dephan_node", "index", "installation"], "filenames": ["cpp_src/packet_handler_mech.rst", "cpp_src/packet_raw.rst", "cpp_src/reciever_socket.rst", "cpp_src/ros_driver.rst", "dephan_node.rst", "index.rst", "installation.rst"], "titles": ["Class pkt_hdl_Mech", "Struct packet", "Class receiver_socket", "Class Driver", "DEPHAN ROS node howto", "Dephan ROS driver documentation", "Dephan ROS driver installation"], "terms": {"defin": [0, 1, 2, 3], "ad": 0, "mech_lidar_driv": [0, 1, 2, 3, 4], "includ": [0, 1, 2, 3], "packet_handler_mech": [0, 5], "hpp": [0, 1, 2, 3, 5], "public": [0, 1, 2, 3], "dephan_ro": [0, 1], "packet": [0, 2, 3, 5], "handl": [0, 5], "raw": [0, 5], "deriv": 0, "from": [0, 3, 5], "base": [0, 1], "function": [0, 2, 3], "raw_packet_t": [0, 1], "pkt": 0, "constructor": [0, 3], "const": [0, 1], "delet": 0, "disabl": 0, "copi": 0, "oper": [0, 3, 4, 5, 6], "assign": 0, "member": [0, 3], "float": 0, "rang": 0, "chanel": 0, "0": 0, "point": 0, "within": 0, "one": [0, 3], "scan": 0, "angl": [0, 3], "raw_pkt": 0, "have": 0, "handel": 0, "static": [0, 1], "attribut": [0, 1], "uint8_t": [0, 1, 2], "magic": 0, "0x68": 0, "": [0, 1, 3], "byte": 0, "int": [0, 2], "125": 0, "number": 0, "photodetect": 0, "unit": 0, "protocol_vers": 0, "0x00": 0, "version": 0, "commun": 0, "protocol": 0, "constexpr": 0, "rad_resolut": 0, "2": [0, 3, 4], "3": [0, 4], "1415": 0, "2250": 0, "resolut": 0, "packet_raw": [1, 5], "class": 1, "subclass": 1, "pkt_hdl_mech": 1, "type": 1, "typedef": 1, "std": [1, 2, 3], "unique_ptr": [1, 3], "unsign": [1, 3], "pkt_len": 1, "1016": 1, "mechan": 1, "lidar": [1, 3, 5], "singl": [1, 2], "length": 1, "reciever_socket": [2, 5], "string": [2, 3], "ip_addr": [2, 3], "port": [2, 3], "construct": 2, "open": 2, "socket": [2, 3], "specif": 2, "ip": [2, 3], "address": [2, 3], "paramet": [2, 3], "config": [2, 4], "devic": [2, 3, 5], "configur": [2, 4], "destruct": 2, "udp": [2, 3, 4], "server": 2, "close": 2, "get_packet": 2, "buf": 2, "len": 2, "get": [2, 5], "data": [2, 3, 4, 5], "buffer": 2, "lenght": 2, "ros_driv": [3, 5], "node": [3, 6], "ro": 3, "topic_nam": 3, "bool": 3, "is_ful": 3, "fals": 3, "mode": [3, 4], "network": 3, "name": 3, "topic": [3, 4, 5], "pointcloud": 3, "v2": 3, "messag": 3, "flag": 3, "true": 3, "pi": 3, "rad": 3, "segment": 3, "per": 3, "18": 3, "pcap_path": 3, "pcap": [3, 4, 5], "construcror": 3, "path": 3, "target": 3, "file": [3, 4, 5], "void": 3, "poll": [3, 5], "poll_ful": 3, "pair": 3, "get_network_param": 3, "getter": 3, "privat": 3, "timer_callback": 3, "timer": 3, "callback": 3, "_poll_udp": 3, "_poll_pcap": 3, "_poll_full_udp": 3, "upd": 3, "_poll_full_pcap": 3, "pill": 3, "receiver_socket": 3, "pointer": 3, "reciev": [3, 5], "read": 3, "tin": [3, 6], "filesniff": 3, "pcap_sniff": 3, "libtin": [3, 6], "shiffer": 3, "long": 3, "_prev_pkt_tmstmp": 3, "store": 3, "timestamp": 3, "time": 3, "correct": 3, "rclcpp": 3, "publish": 3, "sensor_msg": 3, "msg": 3, "pointcloud2": 3, "sharedptr": 3, "pointcloud2_publish": 3, "timerbas": 3, "For": [4, 6], "iron": [4, 6], "you": [4, 5, 6], "can": [4, 5], "specifi": 4, "json": [4, 6], "driver": 4, "find": 4, "default": 4, "both": 4, "respect": 4, "default_pcap_config": 4, "default_udp_config": 4, "creat": 4, "separet": 4, "termin": 4, "In": 4, "tetmin": 4, "1": 4, "noetic": [4, 6], "roscor": 4, "echo": 4, "No": 4, "need": 4, "thi": [4, 5], "rosrun": 4, "mech_driv": 4, "ros2": [4, 6], "run": [4, 5, 6], "path_to_json_config": 4, "rostop": 4, "point_cloud2_data": 4, "now": [4, 6], "should": [4, 5], "see": 4, "stream": 4, "To": 5, "instal": 5, "all": 5, "neccesari": 5, "packag": [5, 6], "setup": [5, 6], "pleas": [5, 6], "follow": [5, 6], "guid": 5, "The": 5, "purpos": 5, "i": 5, "provid": 5, "conveni": 5, "easi": 5, "us": [5, 6], "support": 5, "llc": [5, 6], "There": 5, "ar": [5, 6], "some": 5, "structur": 5, "part": 5, "incapsul": 5, "inform": 5, "about": 5, "contai": 5, "method": 5, "contain": 5, "connect": 5, "also": 5, "test": [5, 6], "your": 5, "describ": 5, "dephan_nod": 5, "cpp": 5, "fot": 5, "wich": 5, "process": 5, "translat": 5, "how": [5, 6], "howto": [5, 6], "wa": 6, "under": 6, "distribut": 6, "select": 6, "ros1": 6, "pcl": 6, "sudo": 6, "apt": 6, "libpcl": 6, "dev": 6, "updat": 6, "convers": 6, "libpcap": 6, "libssl": 6, "cmake": 6, "cd": 6, "git": 6, "clone": 6, "http": 6, "github": 6, "com": 6, "mfontanini": 6, "mkdir": 6, "make": 6, "export": 6, "ld_library_path": 6, "usr": 6, "local": 6, "lib": 6, "nlohmann": 6, "command": 6, "b": 6, "main": 6, "catkin_mak": 6, "sourc": 6, "devel": 6, "bash": 6, "hot": 6, "colcon": 6, "readi": 6, "learn": 6}, "objects": {"": [[3, 0, 1, "_CPPv4N10dephan_ros6DriverE", "dephan_ros::Driver"], [3, 1, 1, "_CPPv4N10dephan_ros6Driver6DriverENSt6stringENSt6stringEb", "dephan_ros::Driver::Driver"], [3, 1, 1, "_CPPv4N10dephan_ros6Driver6DriverENSt6stringEjNSt6stringEb", "dephan_ros::Driver::Driver"], [3, 2, 1, "_CPPv4N10dephan_ros6Driver6DriverENSt6stringEjNSt6stringEb", "dephan_ros::Driver::Driver::ip_addr"], [3, 2, 1, "_CPPv4N10dephan_ros6Driver6DriverENSt6stringENSt6stringEb", "dephan_ros::Driver::Driver::is_full"], [3, 2, 1, "_CPPv4N10dephan_ros6Driver6DriverENSt6stringEjNSt6stringEb", "dephan_ros::Driver::Driver::is_full"], [3, 2, 1, "_CPPv4N10dephan_ros6Driver6DriverENSt6stringENSt6stringEb", "dephan_ros::Driver::Driver::pcap_path"], [3, 2, 1, "_CPPv4N10dephan_ros6Driver6DriverENSt6stringEjNSt6stringEb", "dephan_ros::Driver::Driver::port"], [3, 2, 1, "_CPPv4N10dephan_ros6Driver6DriverENSt6stringENSt6stringEb", "dephan_ros::Driver::Driver::topic_name"], [3, 2, 1, "_CPPv4N10dephan_ros6Driver6DriverENSt6stringEjNSt6stringEb", "dephan_ros::Driver::Driver::topic_name"], [3, 1, 1, "_CPPv4N10dephan_ros6Driver15_poll_full_pcapEv", "dephan_ros::Driver::_poll_full_pcap"], [3, 1, 1, "_CPPv4N10dephan_ros6Driver14_poll_full_udpEv", "dephan_ros::Driver::_poll_full_udp"], [3, 1, 1, "_CPPv4N10dephan_ros6Driver10_poll_pcapEv", "dephan_ros::Driver::_poll_pcap"], [3, 1, 1, "_CPPv4N10dephan_ros6Driver9_poll_udpEv", "dephan_ros::Driver::_poll_udp"], [3, 3, 1, "_CPPv4N10dephan_ros6Driver16_prev_pkt_tmstmpE", "dephan_ros::Driver::_prev_pkt_tmstmp"], [3, 1, 1, "_CPPv4N10dephan_ros6Driver18get_network_paramsEv", "dephan_ros::Driver::get_network_params"], [3, 3, 1, "_CPPv4N10dephan_ros6Driver7ip_addrE", "dephan_ros::Driver::ip_addr"], [3, 3, 1, "_CPPv4N10dephan_ros6Driver7is_fullE", "dephan_ros::Driver::is_full"], [3, 3, 1, "_CPPv4N10dephan_ros6Driver9pcap_pathE", "dephan_ros::Driver::pcap_path"], [3, 3, 1, "_CPPv4N10dephan_ros6Driver12pcap_snifferE", "dephan_ros::Driver::pcap_sniffer"], [3, 3, 1, "_CPPv4N10dephan_ros6Driver21pointcloud2_publisherE", "dephan_ros::Driver::pointcloud2_publisher"], [3, 1, 1, "_CPPv4N10dephan_ros6Driver4pollEv", "dephan_ros::Driver::poll"], [3, 1, 1, "_CPPv4N10dephan_ros6Driver9poll_fullEv", "dephan_ros::Driver::poll_full"], [3, 3, 1, "_CPPv4N10dephan_ros6Driver4portE", "dephan_ros::Driver::port"], [3, 3, 1, "_CPPv4N10dephan_ros6Driver6socketE", "dephan_ros::Driver::socket"], [3, 3, 1, "_CPPv4N10dephan_ros6Driver5timerE", "dephan_ros::Driver::timer"], [3, 1, 1, "_CPPv4N10dephan_ros6Driver14timer_callbackEv", "dephan_ros::Driver::timer_callback"], [1, 0, 1, "_CPPv4N10dephan_ros6packetE", "dephan_ros::packet"], [1, 3, 1, "_CPPv4N10dephan_ros6packet7PKT_LENE", "dephan_ros::packet::PKT_LEN"], [1, 4, 1, "_CPPv4N10dephan_ros6packet12raw_packet_tE", "dephan_ros::packet::raw_packet_t"], [0, 0, 1, "_CPPv4N10dephan_ros12pkt_hdl_MechE", "dephan_ros::pkt_hdl_Mech"], [0, 3, 1, "_CPPv4N10dephan_ros12pkt_hdl_Mech8CHANELLSE", "dephan_ros::pkt_hdl_Mech::CHANELLS"], [0, 3, 1, "_CPPv4N10dephan_ros12pkt_hdl_Mech14RAD_RESOLUTIONE", "dephan_ros::pkt_hdl_Mech::RAD_RESOLUTION"], [0, 3, 1, "_CPPv4N10dephan_ros12pkt_hdl_Mech6anglesE", "dephan_ros::pkt_hdl_Mech::angles"], [0, 3, 1, "_CPPv4N10dephan_ros12pkt_hdl_Mech5magicE", "dephan_ros::pkt_hdl_Mech::magic"], [0, 1, 1, "_CPPv4N10dephan_ros12pkt_hdl_MechaSERK12pkt_hdl_Mech", "dephan_ros::pkt_hdl_Mech::operator="], [0, 1, 1, "_CPPv4N10dephan_ros12pkt_hdl_Mech12pkt_hdl_MechE12raw_packet_t", "dephan_ros::pkt_hdl_Mech::pkt_hdl_Mech"], [0, 1, 1, "_CPPv4N10dephan_ros12pkt_hdl_Mech12pkt_hdl_MechERK12pkt_hdl_Mech", "dephan_ros::pkt_hdl_Mech::pkt_hdl_Mech"], [0, 2, 1, "_CPPv4N10dephan_ros12pkt_hdl_Mech12pkt_hdl_MechE12raw_packet_t", "dephan_ros::pkt_hdl_Mech::pkt_hdl_Mech::pkt"], [0, 3, 1, "_CPPv4N10dephan_ros12pkt_hdl_Mech16protocol_versionE", "dephan_ros::pkt_hdl_Mech::protocol_version"], [0, 3, 1, "_CPPv4N10dephan_ros12pkt_hdl_Mech6rangesE", "dephan_ros::pkt_hdl_Mech::ranges"], [0, 3, 1, "_CPPv4N10dephan_ros12pkt_hdl_Mech7raw_pktE", "dephan_ros::pkt_hdl_Mech::raw_pkt"], [2, 0, 1, "_CPPv4N10dephan_ros15receiver_socketE", "dephan_ros::receiver_socket"], [2, 1, 1, "_CPPv4N10dephan_ros15receiver_socket10get_packetEP7uint8_ti", "dephan_ros::receiver_socket::get_packet"], [2, 2, 1, "_CPPv4N10dephan_ros15receiver_socket10get_packetEP7uint8_ti", "dephan_ros::receiver_socket::get_packet::buf"], [2, 2, 1, "_CPPv4N10dephan_ros15receiver_socket10get_packetEP7uint8_ti", "dephan_ros::receiver_socket::get_packet::len"], [2, 1, 1, "_CPPv4N10dephan_ros15receiver_socket15receiver_socketENSt6stringEi", "dephan_ros::receiver_socket::receiver_socket"], [2, 2, 1, "_CPPv4N10dephan_ros15receiver_socket15receiver_socketENSt6stringEi", "dephan_ros::receiver_socket::receiver_socket::ip_addr"], [2, 2, 1, "_CPPv4N10dephan_ros15receiver_socket15receiver_socketENSt6stringEi", "dephan_ros::receiver_socket::receiver_socket::port"], [2, 1, 1, "_CPPv4N10dephan_ros15receiver_socketD0Ev", "dephan_ros::receiver_socket::~receiver_socket"]]}, "objtypes": {"0": "cpp:class", "1": "cpp:function", "2": "cpp:functionParam", "3": "cpp:member", "4": "cpp:type"}, "objnames": {"0": ["cpp", "class", "C++ class"], "1": ["cpp", "function", "C++ function"], "2": ["cpp", "functionParam", "C++ function parameter"], "3": ["cpp", "member", "C++ member"], "4": ["cpp", "type", "C++ type"]}, "titleterms": {"class": [0, 2, 3], "pkt_hdl_mech": 0, "document": [0, 1, 2, 3, 5], "struct": 1, "packet": 1, "receiver_socket": 2, "driver": [3, 5, 6], "dephan": [4, 5, 6], "ro": [4, 5, 6], "node": [4, 5], "howto": 4, "descript": 4, "usag": [4, 5], "quick": 5, "start": 5, "project": 5, "strucrut": 5, "link": 5, "instal": 6, "support": 6, "platfrorm": 6, "requir": 6, "build": 6}, "envversion": {"sphinx.domains.c": 3, "sphinx.domains.changeset": 1, "sphinx.domains.citation": 1, "sphinx.domains.cpp": 9, "sphinx.domains.index": 1, "sphinx.domains.javascript": 3, "sphinx.domains.math": 2, "sphinx.domains.python": 4, "sphinx.domains.rst": 2, "sphinx.domains.std": 2, "sphinx": 58}, "alltitles": {"Class pkt_hdl_Mech": [[0, "class-pkt-hdl-mech"]], "Class Documentation": [[0, "class-documentation"], [2, "class-documentation"], [3, "class-documentation"]], "Struct packet": [[1, "struct-packet"]], "Struct Documentation": [[1, "struct-documentation"]], "Class receiver_socket": [[2, "class-receiver-socket"]], "Class Driver": [[3, "class-driver"]], "DEPHAN ROS node howto": [[4, "dephan-ros-node-howto"]], "Description": [[4, "description"]], "Usage": [[4, "usage"]], "Dephan ROS driver documentation": [[5, "dephan-ros-driver-documentation"]], "Quick start": [[5, "quick-start"]], "Project strucrute": [[5, "project-strucrute"]], "DEPHAN ROS node usage": [[5, "dephan-ros-node-usage"]], "Quick links": [[5, "quick-links"]], "Dephan ROS driver installation": [[6, "dephan-ros-driver-installation"]], "Supported platfrorms": [[6, "supported-platfrorms"]], "Requirements": [[6, "requirements"]], "Building": [[6, "building"]]}, "indexentries": {"dephan_ros::pkt_hdl_mech (c++ class)": [[0, "_CPPv4N10dephan_ros12pkt_hdl_MechE"]], "dephan_ros::pkt_hdl_mech::chanells (c++ member)": [[0, "_CPPv4N10dephan_ros12pkt_hdl_Mech8CHANELLSE"]], "dephan_ros::pkt_hdl_mech::rad_resolution (c++ member)": [[0, "_CPPv4N10dephan_ros12pkt_hdl_Mech14RAD_RESOLUTIONE"]], "dephan_ros::pkt_hdl_mech::angles (c++ member)": [[0, "_CPPv4N10dephan_ros12pkt_hdl_Mech6anglesE"]], "dephan_ros::pkt_hdl_mech::magic (c++ member)": [[0, "_CPPv4N10dephan_ros12pkt_hdl_Mech5magicE"]], "dephan_ros::pkt_hdl_mech::operator= (c++ function)": [[0, "_CPPv4N10dephan_ros12pkt_hdl_MechaSERK12pkt_hdl_Mech"]], "dephan_ros::pkt_hdl_mech::pkt_hdl_mech (c++ function)": [[0, "_CPPv4N10dephan_ros12pkt_hdl_Mech12pkt_hdl_MechE12raw_packet_t"], [0, "_CPPv4N10dephan_ros12pkt_hdl_Mech12pkt_hdl_MechERK12pkt_hdl_Mech"]], "dephan_ros::pkt_hdl_mech::protocol_version (c++ member)": [[0, "_CPPv4N10dephan_ros12pkt_hdl_Mech16protocol_versionE"]], "dephan_ros::pkt_hdl_mech::ranges (c++ member)": [[0, "_CPPv4N10dephan_ros12pkt_hdl_Mech6rangesE"]], "dephan_ros::pkt_hdl_mech::raw_pkt (c++ member)": [[0, "_CPPv4N10dephan_ros12pkt_hdl_Mech7raw_pktE"]], "dephan_ros::packet (c++ struct)": [[1, "_CPPv4N10dephan_ros6packetE"]], "dephan_ros::packet::pkt_len (c++ member)": [[1, "_CPPv4N10dephan_ros6packet7PKT_LENE"]], "dephan_ros::packet::raw_packet_t (c++ type)": [[1, "_CPPv4N10dephan_ros6packet12raw_packet_tE"]], "dephan_ros::receiver_socket (c++ class)": [[2, "_CPPv4N10dephan_ros15receiver_socketE"]], "dephan_ros::receiver_socket::get_packet (c++ function)": [[2, "_CPPv4N10dephan_ros15receiver_socket10get_packetEP7uint8_ti"]], "dephan_ros::receiver_socket::receiver_socket (c++ function)": [[2, "_CPPv4N10dephan_ros15receiver_socket15receiver_socketENSt6stringEi"]], "dephan_ros::receiver_socket::~receiver_socket (c++ function)": [[2, "_CPPv4N10dephan_ros15receiver_socketD0Ev"]], "dephan_ros::driver (c++ class)": [[3, "_CPPv4N10dephan_ros6DriverE"]], "dephan_ros::driver::driver (c++ function)": [[3, "_CPPv4N10dephan_ros6Driver6DriverENSt6stringENSt6stringEb"], [3, "_CPPv4N10dephan_ros6Driver6DriverENSt6stringEjNSt6stringEb"]], "dephan_ros::driver::_poll_full_pcap (c++ function)": [[3, "_CPPv4N10dephan_ros6Driver15_poll_full_pcapEv"]], "dephan_ros::driver::_poll_full_udp (c++ function)": [[3, "_CPPv4N10dephan_ros6Driver14_poll_full_udpEv"]], "dephan_ros::driver::_poll_pcap (c++ function)": [[3, "_CPPv4N10dephan_ros6Driver10_poll_pcapEv"]], "dephan_ros::driver::_poll_udp (c++ function)": [[3, "_CPPv4N10dephan_ros6Driver9_poll_udpEv"]], "dephan_ros::driver::_prev_pkt_tmstmp (c++ member)": [[3, "_CPPv4N10dephan_ros6Driver16_prev_pkt_tmstmpE"]], "dephan_ros::driver::get_network_params (c++ function)": [[3, "_CPPv4N10dephan_ros6Driver18get_network_paramsEv"]], "dephan_ros::driver::ip_addr (c++ member)": [[3, "_CPPv4N10dephan_ros6Driver7ip_addrE"]], "dephan_ros::driver::is_full (c++ member)": [[3, "_CPPv4N10dephan_ros6Driver7is_fullE"]], "dephan_ros::driver::pcap_path (c++ member)": [[3, "_CPPv4N10dephan_ros6Driver9pcap_pathE"]], "dephan_ros::driver::pcap_sniffer (c++ member)": [[3, "_CPPv4N10dephan_ros6Driver12pcap_snifferE"]], "dephan_ros::driver::pointcloud2_publisher (c++ member)": [[3, "_CPPv4N10dephan_ros6Driver21pointcloud2_publisherE"]], "dephan_ros::driver::poll (c++ function)": [[3, "_CPPv4N10dephan_ros6Driver4pollEv"]], "dephan_ros::driver::poll_full (c++ function)": [[3, "_CPPv4N10dephan_ros6Driver9poll_fullEv"]], "dephan_ros::driver::port (c++ member)": [[3, "_CPPv4N10dephan_ros6Driver4portE"]], "dephan_ros::driver::socket (c++ member)": [[3, "_CPPv4N10dephan_ros6Driver6socketE"]], "dephan_ros::driver::timer (c++ member)": [[3, "_CPPv4N10dephan_ros6Driver5timerE"]], "dephan_ros::driver::timer_callback (c++ function)": [[3, "_CPPv4N10dephan_ros6Driver14timer_callbackEv"]]}})