#include "rl_control/Reference_generator_node.hpp"
#include "rl_control/param_loader.hpp"

/* Constructor */
ReferenceGenerator::ReferenceGenerator(ros::NodeHandle& nh): 
priv_handle("~")
{
    ROS_INFO_STREAM("flag" << flag_LQT);
    switch(flag_LQT)
    {
        case 1:
            loadMatrix(nh, "Am", Am);
            loadVector(nh, "xm", xm);
            loadMatrix(nh, "Cmx", Cmx);
            loadMatrix(nh, "Cmy", Cmy);
            break;
        case 0:
            loadMatrix(nh, "Am_LQTI", Am);
            loadVector(nh, "xm_LQTI", xm);
            loadMatrix(nh, "Cmx_LQTI", Cmx);
            loadMatrix(nh, "Cmy_LQTI", Cmy);
            nh.getParam("yss", yss);

            break;
    }

    ROS_INFO_STREAM("Am" << Am);
    ROS_INFO_STREAM("xm" << xm);
    ROS_INFO_STREAM("Cmx" << Cmx);
    ROS_INFO_STREAM("Cmy" << Cmy);

    
    ROS_DEBUG("Constructor called.");
}

/* Destructor */
ReferenceGenerator::~ReferenceGenerator()
{
}

void ReferenceGenerator::configNode()
{
    configPublishers();
    configService();
    // ROS_INFO("Node configured.");
}

void ReferenceGenerator::configPublishers(){
    ref_cmd_pub = handle.advertise<rl_control::Float64MultiArrayStamped>("/rl_control/ref_generator", 1);
    out_ref_pub = handle.advertise<geometry_msgs::Vector3Stamped>("/rl_control/output_reference", 1);

}

void ReferenceGenerator::configService(){
    client_ref = handle.serviceClient<std_srvs::SetBool>("/rl_control/enable_control");
}

bool ReferenceGenerator::waitForPlant(bool enable) {
    // std_srvs::SetBool srv;
    srv.request.data = enable;

    ROS_INFO("Esperando a planta responder...");
    client_ref.waitForExistence(); // Bloqueia até o serviço aparecer

    if (client_ref.call(srv)) {
        ROS_INFO_STREAM("Resposta do serviço: " << srv.response.message);
        return srv.response.success;
    } else {
        ROS_ERROR("Falha ao chamar serviço enable_control");
        return false;
    }
}

Eigen::Vector3d ReferenceGenerator::Excitation(double& t)
{
    // // white noise
    Eigen::Vector3d noise_white, noise_sin, excitation;

    // noise_white.x() = dist(generator);

    // // sinuidal noise

    // noise_sin.x() =   0.5 * sin(2*M_PI*0.5*t)
    //                 + 0.4 * sin(2*M_PI*2.0*t)
    //                 + 0.3 * sin(2*M_PI*5.0*t)
    //                 + 0.2 * sin(2*M_PI*7.0*t);

    std::default_random_engine generator;
    std::normal_distribution<double> white_dist{0.0, 1.0}; // unit variance, escalar depois
    std::vector<double> sine_freqs = {0.2, 0.5, 1.0, 2.5}; // Hz
    std::vector<double> sine_amps  = {0.12, 0.08, 0.05, 0.03}; // amplitudes
    double exc = 0.0;
    // soma de senóides
    for (size_t i=0;i<sine_freqs.size();++i) {
        exc += sine_amps[i] * sin(2.0*M_PI * sine_freqs[i] * t + 0.0 /*fase opcional*/);
    }
    // ruído branco filtrado (simples: ruído baixo-pass via média móvel)
    double white = white_dist(generator) * 0.05; // escala do ruído
    // opcional: filtro simples (exponential lowpass)
    static double lp_prev = 0.0;
    double alpha = 0.1; // 0..1 (menor = mais filtrado)
    lp_prev = alpha * white + (1.0 - alpha) * lp_prev;
    exc += lp_prev;

    // opcional: adicionar PRBS (simples)
    // static double prbs_last = 0; static double prbs_tlast = 0;
    // if (t - prbs_tlast > prbs_step) { prbs_last = (rand()&1)? +amp : -amp; prbs_tlast = t; } exc += prbs_last;

    excitation.x() = exc;
    
    // ROS_INFO_STREAM("excitation" << excitation);
    return excitation;
}

void ReferenceGenerator::sendRefPos(double h){

    static double t = 0.0;
    t += h;
   
    // output_ref.header.stamp = ros::Time::now();



    if(flag_LQT)
    {
        output_ref.vector.x = (Cmx * xm).value(); 
        output_ref.vector.y = (Cmy * xm).value(); 
        ref_msg.data.data.resize(5);
        for (int i = 0; i < 5; i++) 
        {
            ref_msg.data.data[i] = xm(i);  
        }
    }
    else
    {
        output_ref.vector.x = (Cmx * xm).value() + yss;
        output_ref.vector.y = (Cmy * xm).value() + yss;
        ref_msg.data.data.resize(4);
        for (int i = 0; i < 4; i++) 
        {
            ref_msg.data.data[i] = xm(i);  
        }
    }
    
    // set timestamp for the reference message
    ref_msg.header.stamp = ros::Time::now();

    out_ref_pub.publish(output_ref);
    ref_cmd_pub.publish(ref_msg);

    xm = Am * xm;
    
}

int main(int argc, char **argv){

    ros::init(argc, argv, "Reference_generator_node");
    ROS_INFO("This node has started.");

    ros::NodeHandle nh; 
    ReferenceGenerator ref_gen(nh);

    ref_gen.configNode();

    // === Chama o serviço apenas uma vez ===
    if (!ref_gen.waitForPlant(true)) {
        ROS_ERROR("Não foi possível habilitar o controle. Encerrando nó.");
        return -1;  // Finaliza se falhar
    }


    ROS_INFO("Controle habilitado com sucesso! Iniciando geração de referência...");

    ros::Time start_time = ros::Time::now();

    ros::Rate sampling_rate(50);    // Hertz
    while(ros::ok()){

        double h = 1.0/50.0;

        ros::spinOnce();
        ref_gen.sendRefPos(h);

        sampling_rate.sleep();
    }
}
