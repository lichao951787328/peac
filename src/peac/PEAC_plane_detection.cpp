#include <peac/PEAC_plane_detection.hpp>
#include <pcl/common/transforms.h>
std::map<std::string, std::string> ini;
template<class T>
T iniGet(std::string key, T default_value) {
	std::map<std::string, std::string>::const_iterator itr=ini.find(key);
	if(itr!=ini.end()) {
		std::stringstream ss;
		ss<<itr->second;
		T ret;
		ss>>ret;
		return ret;
	}
	return default_value;
}

template<> std::string iniGet(std::string key, std::string default_value) {
	std::map<std::string, std::string>::const_iterator itr=ini.find(key);
	if(itr!=ini.end()) {
		return itr->second;
	}
	return default_value;
}

void iniLoad(std::string iniFileName) {
	std::ifstream in(iniFileName);
	if(!in.is_open()) {
		std::cout<<"[iniLoad] "<<iniFileName<<" not found, use default parameters!"<<std::endl;
		return;
	}
	while(in) {
		std::string line;
		std::getline(in, line);
		if(line.empty() || line[0]=='#') continue;
		std::string key, value;
		size_t eqPos = line.find_first_of("=");
		if(eqPos == std::string::npos || eqPos == 0) {
			std::cout<<"[iniLoad] ignore line:"<<line<<std::endl;
			continue;
		}
		key = line.substr(0,eqPos);
		value = line.substr(eqPos+1);
		std::cout<<"[iniLoad] "<<key<<"=>"<<value<<std::endl;
		ini[key]=value;
	}
}

plane_detection::plane_detection()
{

}

void plane_detection::initial(string parameter_address)
{
	if (parameter_address.empty())
	{
		cout<<"parameter address is error"<<endl;
	}
	
    showWindow = iniGet("showWindow", true);
    iniLoad(parameter_address);
    unitScaleFactor = iniGet<double>("unitScaleFactor", 1.0f);
	outputDir = iniGet<std::string>("outputDir", ".");
    std::string cmd="mkdir -p "+outputDir;
    system(cmd.c_str());
    std::cout << "create:" << outputDir << std::endl;

    pf.minSupport = iniGet<int>("minSupport", 3000);
	pf.windowWidth = iniGet<int>("windowWidth", 10);
	pf.windowHeight = iniGet<int>("windowHeight", 10);
	pf.doRefine = iniGet<int>("doRefine", 1) != 0;
    pf.params.initType = (ahc::InitType)iniGet("initType", (int)pf.params.initType);
    pf.params.stdTol_merge = iniGet("stdTol_merge", pf.params.stdTol_merge);
	pf.params.stdTol_init = iniGet("stdTol_init", pf.params.stdTol_init);
	pf.params.depthSigma = iniGet("depthSigma", pf.params.depthSigma);
    pf.params.depthAlpha = iniGet("depthAlpha", pf.params.depthAlpha);
	pf.params.depthChangeTol = iniGet("depthChangeTol", pf.params.depthChangeTol);
    pf.params.z_near = iniGet("z_near", pf.params.z_near);
	pf.params.z_far = iniGet("z_far", pf.params.z_far);
	pf.params.angle_near = MACRO_DEG2RAD(iniGet("angleDegree_near", MACRO_RAD2DEG(pf.params.angle_near)));
	pf.params.angle_far = MACRO_DEG2RAD(iniGet("angleDegree_far", MACRO_RAD2DEG(pf.params.angle_far)));
	pf.params.similarityTh_merge = std::cos(MACRO_DEG2RAD(iniGet("similarityDegreeTh_merge", MACRO_RAD2DEG(pf.params.similarityTh_merge))));
	pf.params.similarityTh_refine = std::cos(MACRO_DEG2RAD(iniGet("similarityDegreeTh_refine", MACRO_RAD2DEG(pf.params.similarityTh_refine))));
}

void plane_detection::detect(pcl::PointCloud<pcl::PointXYZ> & pc)
{
	// pc.height = 480;
	// pc.width = 680;
	planes_info.clear();
	planes.clear();
	
	pc.points.resize(pc.height * pc.width);
	std::cout<<pc.width<<" "<<pc.height<<std::endl;
	std::cout<<"unitScaleFactor: "<<unitScaleFactor<<std::endl;
    pcl::transformPointCloud<pcl::PointXYZ>(pc, pc, Eigen::Affine3f(Eigen::UniformScaling<float>((float)unitScaleFactor)));
    // processOneFrame(cloud, outputFilePrefix);
    cv::Mat seg(pc.height, pc.width, CV_8UC3);
	std::cout<<"start"<<std::endl;
    ImageXYZ Ixyz(pc);
	std::cout<<"initial"<<std::endl;
	ahc::utils::Timer timer(1000);
	timer.tic();
	pf.run(&Ixyz, 0, &seg);
	for (auto & plane_info : pf.extractedPlanes)
	{
		Eigen::Vector3d normal(plane_info->normal[0], plane_info->normal[1], plane_info->normal[2]);
		// 在进行提取时，会把平面按照unitScaleFactor放大，这个中点需要恢复
		Eigen::Vector3d center(plane_info->center[0]/unitScaleFactor, plane_info->center[1]/unitScaleFactor, plane_info->center[2]/unitScaleFactor);
		planes_info.emplace_back(planeInfo(normal, center));
	}
	planes = pf.planes;
	
	double process_ms=timer.toc();
	std::cout<<process_ms<<" ms"<<std::endl;
    
    // cv::cvtColor(seg,seg,cv::COLOR_RGB2BGR);
	result = seg;
    if (showWindow)
    {
        // std::string outputFilePrefix = outputDir+std::to_string(process_index);
		cv::imshow("seg", seg);
		cv::waitKey(10);
        // cv::imwrite(outputFilePrefix+".seg.png", seg);
	    // std::cout<<"output: "<<outputFilePrefix<<".seg.png"<<std::endl;
        // process_index++;
    }
	
    // if(showWindow) 
    // {
	// 	//show frame rate
    //     cv::namedWindow("seg");
	// 	std::stringstream stext;
	// 	stext<<"Frame Rate: "<<(1000.0/process_ms)<<"Hz";
	// 	cv::putText(seg, stext.str(), cv::Point(15,15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255,1));
	// 	cv::imshow("seg", seg);
	// 	cv::waitKey(10);
	// }
}

plane_detection::~plane_detection()
{
}