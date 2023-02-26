#include "multi_bspline_opt/bspline_opt.h"

namespace my_planner

{
    UniformBspline::UniformBspline(const int &p,  const int &n, const double &beta, const int &D, 
                                                 const Eigen::MatrixXd &s_ini, const Eigen::MatrixXd &s_ter)
    {
        initUniformBspline(p, n,beta, D, s_ini, s_ter); 
    }

    UniformBspline::~UniformBspline() {}

    void UniformBspline::initUniformBspline(const int &p,  const int &n, const double &beta, const int &D, 
                                                 const Eigen::MatrixXd &s_ini, const Eigen::MatrixXd &s_ter)
    {
        p_ = p; 
        n_ = n-1;
        beta_ = beta;
        D_ =D;
        m_ = p_+n_+1;
        u_ = Eigen::VectorXd::Zero(m_ + 1); //u0 ~ um 共m+1个
        control_points_ = Eigen::MatrixXd::Zero(n_+1,D_);
        for(int i = 0; i<=m_; i++)
        {
            u_(i) = i;
        }
        s_ini_ = s_ini;
        s_ter_ = s_ter;
        setIniTerMatrix();
        getAvailableSrange();
        getAvailableTrange();
        getInterval();
    }

    void UniformBspline::setIniTerMatrix()
    {
        A_ini.resize(3,3);
        A_ter.resize(3,3);
        A_ini << 1.0/6, 2.0/3, 1.0/6,
                        -1.0/2*beta_, 0.0*beta_, 1.0/2*beta_,
                        1.0*beta_*beta_,-2.0*beta_*beta_,1.0*beta_*beta_;
        A_ter<<1.0/6, 2.0/3, 1.0/6,
                        -1.0/2*beta_, 0.0*beta_, 1.0/2*beta_,
                        1.0*beta_*beta_,-2.0*beta_*beta_,1.0*beta_*beta_;
    }

    void UniformBspline::setControlPoints(const Eigen::MatrixXd &ctrl_points)
    {
        control_points_ = ctrl_points;
    }
    
    Eigen::MatrixXd UniformBspline::getTrajectory(const Eigen::VectorXd &t)
    {
        double u_probe;
        int t_size = t.size();
        Eigen::MatrixXd trajectory(t_size,D_);
        for (size_t i = 0; i < t_size; i++)
        {
            //map t(i) to uniform knot vector
            u_probe = t(i) * beta_ + u_(p_);
            trajectory.row(i) = singleDeboor(u_probe); 
        }
        return trajectory;
    }

     Eigen::Vector2d UniformBspline::singleDeboor(const double &u_probe)//the deboor's algorithm
     {  
        //bound the u_probe
        double u_probe_;
        int k;
        u_probe_ = min(max( u_(p_) , u_probe), u_(m_-p_));
        k = p_;
        while(true)
        {
            if(u_(k+1)>=u_probe_)
                break;
            k = k+1;
        }
        // t(t_ctt) is maped to knot segment u_k ~ u_{k+1}
        // there are at most p+1 basis functions N_k-p,p(u), N_k-p+1,p(u),..., N_k,p(u) non-zero on knot span [uk,uk+1)
        //the effective control points are P_k-p ~ P_k
        // since MATLAB index start from 1 instead of 0
        // The effective control points are
        double alpha;
        Eigen::MatrixXd d(p_+1,2);
        d = control_points_.block(k-p_,0,p_+1,2);// c++这里是从0行0列开始
        for (size_t i = 0; i < p_; i++)
        {
            for (size_t j = p_; j > i; j--)
            {
                alpha = (u_probe_ - u_(j+k-p_)) /(u_(j+k-i) - u_(j+k-p_)); 
                d.row(j) = (1 - alpha)*d.row(j-1) + alpha*d.row(j);
            }          
        }

            Eigen::Vector2d value;
            value = d.row(p_);
            return value;
     }

    void UniformBspline::getAvailableSrange()
    {
        s_range = {u_(p_),u_(m_-p_)};
    }

   double UniformBspline::getAvailableTrange()
    {
        t_range = {0/beta_, (u_(m_-p_)-u_(p_))/beta_};
       double  t_max =  (u_(m_-p_)-u_(p_))/beta_;
        return t_max;
    }

    void UniformBspline::getInterval()
    {
        interval_ = (u_(1) - u_(0))/beta_;
    }

    void UniformBspline::getT(const int &trajSampleRate)
    {
        time_.resize((t_range(1)-t_range(0))*trajSampleRate+1);
        
        for (size_t i = 0; i < time_.size(); i++)
        {
            time_(i) = t_range(0) + i*(1.0/trajSampleRate);
        }
    }

    UniformBspline UniformBspline::getDerivative()
    {     
            UniformBspline spline(p_,n_,beta_,D_,s_ini_,s_ter_);
            spline.p_ = spline.p_ -1;
            spline.m_ = spline.p_ +spline.n_ +1;
            spline.u_.resize(u_.size()-2);
            spline.u_ = u_.segment(1,m_-1);//从第2个元素开始的m-1个元素
            spline.control_points_.resize(control_points_.rows()-1,D_);
            for (size_t i = 0; i < spline.control_points_.rows(); i++)
            {
                spline.control_points_.row(i) = spline.beta_*(control_points_.row(i+1) - control_points_.row(i));
            } 
            spline.time_ = time_;
            return spline;
    }

    Eigen::VectorXd UniformBspline::getBoundConstraintb()
    {
        int nm = (n_+1)*D_;
        Eigen::VectorXd b= Eigen::VectorXd::Zero(nm);
        Eigen::MatrixXd tmp1(3,2);//前三个控制点的值
        Eigen::MatrixXd tmp2(3,2);//末尾三个控制点的值
        // solve Ax = b
        tmp1 = A_ini.colPivHouseholderQr().solve(s_ini_);
        tmp2 = A_ter.colPivHouseholderQr().solve(s_ter_);
         for (size_t j = 0; j< D_; j++)
        {
            for (size_t i = 0; i < 3; i++)
            {
                b(i+j*(n_+1)) = tmp1(i,j);
                b((j+1)*(n_+1)-i-1) = tmp2(3-i-1,j);
            }      
        }    
        return b;   
    }

/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/


    bspline_optimizer::bspline_optimizer(const std::vector<Eigen::Vector2d> &path, const int&Dim,  const int&p)
    {
        path_.clear();
        path_ = path;
        Dim_  = Dim;
        p_order_ = p;
        cps_num_ = path.size() + 2*p_order_ -2;
        // drone_pos_world=drone_pos_world_;
        // planning_horizen_ = planning_horizen;
    }
    bspline_optimizer::bspline_optimizer(const int&Dim,  const int&p, const double &dist)
    {
        Dim_  = Dim;
        p_order_ = p;
        cps_num_ = 2*p_order_+floor(dist/1.0);
    }
    bspline_optimizer::~bspline_optimizer(){}
    
    void bspline_optimizer::setOptParam(const double lambda1,const double lambda2,const double lambda3,const double lambda4,
                                                                                    const double safe_dist, const double swarm_clearance)
    {
            lambda1_ = lambda1;
            lambda2_ = lambda2;
            lambda3_ = lambda3;
            lambda4_ = lambda4;
            safe_distance_ = safe_dist;
            swarm_clearance_ = swarm_clearance;
    }
        void bspline_optimizer::setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr)
     { 
        swarm_trajs_ = swarm_trajs_ptr;
         }
    void bspline_optimizer::setVelAcc(const double vel, const double acc)
    {
            max_vel_ = vel;
            max_acc_ = acc;
    }
  void bspline_optimizer::setSmoothParam(const double lambda1,const double lambda2,
                                                            const double vel, const double acc)
    {
            lambda1_ = lambda1;
            lambda2_ = lambda2;
            max_vel_ = vel;
            max_acc_ = acc;
    }
    void bspline_optimizer::setEsdfMap(const Eigen::MatrixXd &esdf_map)
    {
        if(esdf_map.size()==0)
        {
          esdf_map_.setZero(1000,1000); 
          cout<<" esdf map is empty!!!!!!"<<endl;
        }
        else
        esdf_map_ = esdf_map;
    }

    void bspline_optimizer::setMapParam(const double &origin_x,const double &origin_y, const double &map_resolution,
                                                                                    const double &start_x, const double &start_y)
    {
        origin_x_ = origin_x;
        origin_y_ = origin_y;
        map_resolution_ = map_resolution;
        startPoint_x = start_x;
        startPoint_y = start_y;
    }

    void bspline_optimizer::setSplineParam(const UniformBspline &u)
    {
        u_ = u;
        bspline_interval_  = u.interval_;
        beta_ = u.beta_;
        control_points_.resize(cps_num_,Dim_);
        Eigen::VectorXd beq_bound = u_.getBoundConstraintb();
        
        for (size_t i = 0; i < Dim_; i++)
        {
                for (size_t j = 0; j < p_order_; j++)
                {
                     control_points_(j,i) = beq_bound(i*cps_num_+j);
                     control_points_((1)*cps_num_-j-1,i) = beq_bound((i+1)*cps_num_-j-1);//BUG!!!!
                }
        }
            
        for (size_t i = 0; i < cps_num_-2*p_order_; i++)
        {
            control_points_.row(i+p_order_) = path_[i+1];

        }
    }
        void bspline_optimizer::initialControlPoints(UniformBspline u)
        {
        control_points_.setZero(cps_num_,Dim_);
        Eigen::VectorXd beq_bound = u.getBoundConstraintb();
        for (size_t i = 0; i < Dim_; i++)
        {
        for (size_t j = 0; j < p_order_; j++)
        {
        control_points_(j,i) = beq_bound(i*cps_num_+j);
        control_points_(cps_num_-j-1,i) = beq_bound((i+1)*cps_num_-j-1);
        }
        }
        int insert_num = cps_num_-2*p_order_;
        Eigen::Vector2d start_pos = control_points_.row(p_order_-1);
        Eigen::Vector2d end_pos = control_points_.row(cps_num_-p_order_);
        for (size_t i = 0; i < cps_num_-2*p_order_; i++)
        {
        control_points_(i+p_order_,0) = start_pos(0)+(end_pos(0)-start_pos(0))/(insert_num+1)*(i+1) ;
        control_points_(i+p_order_,1) = start_pos(1)+(end_pos(1)-start_pos(1))/(insert_num+1)*(i+1) ;
        }
       // cout<<control_points_<<endl;
        }
    void bspline_optimizer::calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                                            Eigen::MatrixXd &gradient, bool falg_use_jerk /* = true*/)
    {
        cost = 0.0;
        if (falg_use_jerk)
        {
            Eigen::Vector2d jerk, temp_j;

            for (int i = 0; i < q.cols() - 3; i++)
            {
                /* evaluate jerk */
                jerk = q.col(i + 3) - 3 * q.col(i + 2) + 3 * q.col(i + 1) - q.col(i);
                cost += jerk.squaredNorm();
                temp_j = 2.0 * jerk;
                /* jerk gradient */
                gradient.col(i + 0) += -temp_j;
                gradient.col(i + 1) += 3.0 * temp_j;
                gradient.col(i + 2) += -3.0 * temp_j;
                gradient.col(i + 3) += temp_j;
            }
        }
        else
        {
            Eigen::Vector2d acc, temp_acc;

            for (int i = 0; i < q.cols() - 2; i++)
            {
                /* evaluate acc */
                acc = q.col(i + 2) - 2 * q.col(i + 1) + q.col(i);
                cost += acc.squaredNorm();
                temp_acc = 2.0 * acc;
                /* acc gradient */
                gradient.col(i + 0) += temp_acc;
                gradient.col(i + 1) += -2.0 * temp_acc;
                gradient.col(i + 2) += temp_acc;
            }
        }
    }
    void  bspline_optimizer::calcSwarmCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
  {
    cost = 0.0;
    // ROS_ERROR(" RECIEVED SWARM!");
    // return;
     if (swarm_trajs_== NULL ||swarm_trajs_->size()==0)
     {
        // ROS_ERROR("NOT RECIEVED SWARM!");
      return;
     }
    //  return;
    //  ROS_ERROR(" RECIEVED SWARM!");
    int end_idx = q.cols() - p_order_ - (double)(q.cols() - 2 * p_order_) * 1.0 / 3.0; // Only check the first 2/3 points
    // cout<<"end_idx:"<<end_idx<<endl;
    const double CLEARANCE = swarm_clearance_ * 2;
    double t_now = ros::Time::now().toSec();
    constexpr double a = 2.0, b = 1.0, inv_a2 = 1 / a / a, inv_b2 = 1 / b / b;

    for (int i =p_order_; i < end_idx; i++)
    {
      double glb_time = t_now + ((double)(p_order_  - 1) / 2 + (i - p_order_+ 1)) * bspline_interval_;

      for (size_t id = 0; id < swarm_trajs_->size(); id++)
      {
        if ( swarm_trajs_->at(id).drone_id != (int)id||swarm_trajs_->at(id).drone_id == drone_id_)
        {
        //   ROS_ERROR("  ID ERROR!");
          continue;
        } 
        // cout<<"swarm_trajs_"<<endl;
        double traj_i_satrt_time = swarm_trajs_->at(id).start_time_.toSec();
        //  cout<<"traj_i_satrt_time:"<<traj_i_satrt_time<<endl;
        //  cout<<"duration:"<<swarm_trajs_->at(id).duration_<<endl;
        //  cout<<"glb_time:"<<glb_time<<endl;
        if (glb_time < traj_i_satrt_time + swarm_trajs_->at(id).duration_ )
        {
          // int number;
          // number =floor(( glb_time - traj_i_satrt_time) ) * 50;
          // number =min(swarm_trajs_->at(id).traj_.size, max(number , 0) );
          // Eigen::Vector2d swarm_prid = swarm_trajs_->at(id).traj_[number];
        //   cout<<"start swarm!"<<endl;
          
           Eigen::Vector2d swarm_prid = swarm_trajs_->at(id).position_traj_.singleDeboor(glb_time - traj_i_satrt_time);
        //    ROS_ERROR("START COMPUTE!");
           
        //   Eigen::Vector2d swarm_prid  = swarm_prid1.head(2);
          Eigen::Vector2d uav_control_vec =  control_points_.row(i);
          Eigen::Vector2d dist_vec = uav_control_vec- swarm_prid;
          double ellip_dist = sqrt( (dist_vec(0) * dist_vec(0) + dist_vec(1) * dist_vec(1)) * inv_b2);
          double dist_err = CLEARANCE - ellip_dist;
// cout<<"col："<<control_points_.rows()<<endl;
          Eigen::Vector2d dist_grad =uav_control_vec - swarm_prid;
          Eigen::Vector2d Coeff;
          Coeff(0) = -2 * (CLEARANCE / ellip_dist - 1) * inv_b2;
          Coeff(1) = Coeff(0);
//           Coeff(2) = -2 * (CLEARANCE / ellip_dist - 1) * inv_a2;

          if (dist_err < 0)
          {
            /* do nothing */
          }
          else
          {
            cost += pow(dist_err, 2);
            gradient.col(i) += (Coeff.array() * dist_grad.array()).matrix();
          }

          if (min_ellip_dist_ > dist_err)
          {
            min_ellip_dist_ = dist_err;
          }
        }
      }
    }
  }
  void bspline_optimizer::calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                                                        Eigen::MatrixXd &gradient)
   {
    cost = 0.0;
    /* abbreviation */
    double ts, /*vm2, am2, */ ts_inv2;
    // vm2 = max_vel_ * max_vel_;
    // am2 = max_acc_ * max_acc_;

    ts = bspline_interval_;
    ts_inv2 = 1 / ts / ts;
    /* velocity feasibility */
    for (int i = 0; i < q.cols() - 1; i++)
    {
      Eigen::Vector2d vi = (q.col(i + 1) - q.col(i)) / ts;
      for (int j = 0; j < 2; j++)
      {
        if (vi(j) > max_vel_)
        {
          cost += pow(vi(j) - max_vel_, 2) * ts_inv2; // multiply ts_inv3 to make vel and acc has similar magnitude

          gradient(j, i + 0) += -2 * (vi(j) - max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) - max_vel_) / ts * ts_inv2;
        }
        else if (vi(j) < -max_vel_)
        {
          cost += pow(vi(j) + max_vel_, 2) * ts_inv2;

          gradient(j, i + 0) += -2 * (vi(j) + max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) + max_vel_) / ts * ts_inv2;
        }
        else
        {
          /* code */
        }
      }
    }
    /* acceleration feasibility */
    for (int i = 0; i < q.cols() - 2; i++)
    {
      Eigen::Vector2d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

      for (int j = 0; j < 2; j++)
      {
        if (ai(j) > max_acc_)
        {
          cost += pow(ai(j) - max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) - max_acc_) * ts_inv2;
        }
        else if (ai(j) < -max_acc_)
        {
          cost += pow(ai(j) + max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) + max_acc_) * ts_inv2;
        }
        else
        {
          /* code */
        }
      }

    }
    }
    
    void bspline_optimizer::calcEsdfCost(const Eigen::MatrixXd &q, double &cost,
                                                        Eigen::MatrixXd &gradient)
    {
        cost = 0.0;
        double  dist;
        Eigen::Vector2d dist_grad;
        Eigen::Vector2d tmp_vel;

        for (int i = p_order_; i < q.cols()-p_order_; i++) 
        {
            if(q(0,i)<-40||q(0,i)>40||q(1,i)<-40||q(1,i)>40)
            {
                cout<<"unreasonable control points"<<endl;
            }
            else{
                dist = calcDistance(q.col(i));
                dist_grad = calcGrad(q.col(i));
                if (dist_grad.norm() > 1e-4) dist_grad.normalize();
                if (dist < safe_distance_) 
                {
                    cost += pow(dist - safe_distance_, 2);
                    gradient.col(i) += 2.0 * (dist - safe_distance_) * dist_grad;     
                }
            }

        }   

    }
    double bspline_optimizer::calcDistance(const Eigen::MatrixXd &q)
    {
        double dist;
        Eigen::Vector2d p(q(0,0),q(1,0));//存入两个控制点
        Eigen::Vector2d diff;
        Eigen::Vector2d sur_pts[2][2];//4个邻居点
        getSurroundPts(p,sur_pts, diff);
        double dists[2][2];
        getSurroundDistance(sur_pts, dists);
        interpolateBilinearDist(dists, diff, dist);
        return dist;
    }

    void bspline_optimizer::interpolateBilinearDist(double values[2][2], const Eigen::Vector2d& diff, 
                                                                                                                                                                double& dist)
    {
        //二线性插值
        double c00 = values[0][0];
        double c01 = values[0][1];
        double c10 = values[1][0];
        double c11 = values[1][1];
        double tx = diff(0);
        double ty = diff(1);
        
        double nx0 = lerp(c00,c10,ty);
        double nx1 = lerp(c01,c11,ty);
        double ny0 = lerp(c00,c01,tx);
        double ny1 = lerp(c10,c11,tx);

        dist = lerp(ny0,ny1,ty);
    }
    Eigen::Vector2d bspline_optimizer::calcGrad(const Eigen::MatrixXd &q)
    {
        Eigen::Vector2d p(q(0,0),q(1,0));//存入两个控制点
        Eigen::Vector2d dist_grad;
        Eigen::Vector2d diff;
        Eigen::Vector2d sur_pts[2][2];//4个邻居点
        getSurroundPts(p,sur_pts, diff);

        double dists[2][2];
        getSurroundDistance(sur_pts, dists);

        interpolateBilinear(dists, diff, dist_grad);

        return dist_grad;
    }

    void bspline_optimizer::getSurroundPts(const Eigen::Vector2d& pos, Eigen::Vector2d pts[2][2], Eigen::Vector2d& diff)
    {
        double dist_x = pos(0) - startPoint_x;
        double dist_y = startPoint_y - pos(1);
        diff(0) = fmod(dist_x,map_resolution_);
        diff(1) = fmod(dist_y,map_resolution_);

        Eigen::Vector2d curr_index;//用这个索引找到最左上角的点，并记为 p(0,0);
        curr_index<< floor(dist_y/map_resolution_),floor(dist_x/map_resolution_);
        for (size_t i = 0; i < 2; i++)
        {
            for (size_t j = 0; j < 2; j++)
            {       
                Eigen::Vector2d tmp_index(curr_index(0)+i,curr_index(1)+j);
                pts[i][j] = tmp_index;
            }
        }
    }
    void bspline_optimizer::getSurroundDistance(Eigen::Vector2d pts[2][2], double dists[2][2])
    {
        for (size_t i = 0; i < 2; i++)
        {
            for (size_t j = 0; j < 2; j++)
            {
                Eigen::Vector2d tmp_index = pts[i][j];
                dists[i][j] = esdf_map_(tmp_index(0),tmp_index(1));   
            }
        }
    }
    void bspline_optimizer::interpolateBilinear(double values[2][2], const Eigen::Vector2d& diff, Eigen::Vector2d& grad)
    {
        //二线性插值
        double c00 = values[0][0];
        double c01 = values[0][1];
        double c10 = values[1][0];
        double c11 = values[1][1];
        double tx = diff(0);
        double ty = diff(1);
        
        double nx0 = lerp(c00,c10,ty);
        double nx1 = lerp(c01,c11,ty);
        double ny0 = lerp(c00,c01,tx);
        double ny1 = lerp(c10,c11,tx);

        grad(0) = (nx1- nx0)/map_resolution_;
        grad(1) = (ny0- ny1)/map_resolution_;
    }

    void bspline_optimizer::combineCost( const std::vector<double>& x,Eigen::MatrixXd &grad,double &f_combine)
    {
        Eigen::MatrixXd control_points = control_points_.transpose();
        for (size_t j = 0; j < cps_num_-2*p_order_; j++)
            {
              for (size_t i = 0; i < Dim_; i++)
                {
                    control_points(i,j+p_order_) = x[j*Dim_+i];
                } 
            }
        f_combine = 0.0;
        Eigen::MatrixXd grad2D; 
        grad2D.resize(Dim_,cps_num_);
        grad2D.setZero(Dim_,cps_num_);//初始化梯度矩阵

        double f_smoothness, f_length, f_distance, f_feasibility, f_swarm ;
        Eigen::MatrixXd g_smoothness_ = Eigen::MatrixXd::Zero(Dim_, cps_num_);
        Eigen::MatrixXd g_feasibility_ = Eigen::MatrixXd::Zero(Dim_, cps_num_);
        Eigen::MatrixXd g_distance_ = Eigen::MatrixXd::Zero(Dim_, cps_num_);
         Eigen::MatrixXd g_swarm_ = Eigen::MatrixXd::Zero(Dim_, cps_num_);
        f_smoothness  = f_feasibility =  f_distance = f_swarm = 0.0;
        // cout<< control_points.transpose()<<endl;
        calcSmoothnessCost(control_points, f_smoothness, g_smoothness_);
    // cout<<"====================calcSmoothnessCost"<<endl;
        calcFeasibilityCost(control_points,f_feasibility,g_feasibility_);
    // cout<<"====================calcFeasibilityCost"<<endl;
        calcEsdfCost(control_points,f_distance,g_distance_);
    // cout<<"====================calcEsdfCost"<<endl;
       calcSwarmCost(control_points,f_swarm,g_swarm_);
        f_combine = lambda1_ * f_smoothness + lambda2_*f_feasibility + lambda3_*f_distance + lambda4_*f_swarm;
        ROS_ERROR("f_swarm: %f",f_swarm);
        grad2D = lambda1_*g_smoothness_ + lambda2_ * g_feasibility_ +lambda3_ * g_distance_+lambda4_ * g_swarm_ ;
        grad = grad2D.block(0,p_order_,Dim_,cps_num_-2*p_order_);//起点  块大小
    }
    void bspline_optimizer::combineCostSmooth( const std::vector<double>& x,Eigen::MatrixXd &grad,double &f_combine)
    {
        Eigen::MatrixXd control_points = control_points_.transpose();
        //cout<<control_points<<endl;
        for (size_t j = 0; j < cps_num_-2*p_order_; j++)
            {
              for (size_t i = 0; i < Dim_; i++)
                {
                    control_points(i,j+p_order_) = x[j*Dim_+i];
                } 
            }
        f_combine = 0.0;
        Eigen::MatrixXd grad2D; 
        grad2D.resize(Dim_,cps_num_);
        grad2D.setZero(Dim_,cps_num_);//初始化梯度矩阵

        double f_smoothness, f_feasibility;
        Eigen::MatrixXd g_smoothness_ = Eigen::MatrixXd::Zero(Dim_, cps_num_);
        f_smoothness  =  0.0;
        calcSmoothnessCost(control_points, f_smoothness, g_smoothness_);
        grad2D = lambda1_*g_smoothness_ ;
        grad = grad2D.block(0,p_order_,Dim_,cps_num_-2*p_order_);//起点  块大小
    }
    double bspline_optimizer::costFunction(const std::vector<double>& x, std::vector<double>& grad,
                                                                                         void* func_data)
    {
        bspline_optimizer* opt = reinterpret_cast<bspline_optimizer*>(func_data);
        Eigen::MatrixXd grad_matrix;
        double cost;
        opt->combineCost(x,grad_matrix,cost);
        opt->iter_num_++;

        for (size_t i = 0; i < grad_matrix.cols(); i++)
            {
                for (size_t j = 0; j <opt->Dim_; j++)
                {
                    // grad.push_back(grad_matrix(j,i)) ;
                    grad[i*opt->Dim_+j] = grad_matrix(j,i);
                }    
            } 
        /* save the min cost result */
        if (cost < opt->min_cost_) {
                opt->min_cost_     = cost;
                opt->best_variable_ = x;
            }
        return cost;
    }
    double bspline_optimizer::costFunctionSmooth(const std::vector<double>& x, std::vector<double>& grad,
                                                                                         void* func_data)
    {
        bspline_optimizer* opt = reinterpret_cast<bspline_optimizer*>(func_data);
        Eigen::MatrixXd grad_matrix;
        double cost;
        opt->combineCostSmooth(x,grad_matrix,cost);
        opt->iter_num_++;

        for (size_t i = 0; i < grad_matrix.cols(); i++)
            {
                for (size_t j = 0; j <opt->Dim_; j++)
                {
                    // grad.push_back(grad_matrix(j,i)) ;
                    grad[i*opt->Dim_+j] = grad_matrix(j,i);
                }    
            } 
        /* save the min cost result */
        if (cost < opt->min_cost_) {
                opt->min_cost_     = cost;
                opt->best_variable_ = x;
            }
        return cost;
    }


     void bspline_optimizer::optimize()
    {
            /* initialize solver */
            // cout << "/* initialize solver */"<<endl;
            iter_num_        = 0;
            min_cost_        = std::numeric_limits<double>::max();
            variable_num = (cps_num_-2*p_order_)*Dim_;
            nlopt::opt opt(nlopt::algorithm(nlopt::LD_LBFGS),variable_num);
            opt.set_min_objective(bspline_optimizer::costFunction,this);
            opt.set_maxeval(200);
            opt.set_maxtime(0.02);
            opt.set_xtol_rel(1e-5);
            vector<double> lb(variable_num), ub(variable_num);
            vector<double> q(variable_num);

            for (size_t j = 0; j < cps_num_-2*p_order_; j++)
            {
               for (size_t i = 0; i < Dim_; i++)
                {
                    q[j*Dim_+i] = control_points_(j+p_order_,i);
                }
            }

            const double  bound = 10.0;
            for (size_t i = 0; i <variable_num; i++)
            {
                    lb[i]  = q[i]-bound;
                    ub[i] = q[i]+bound;      
            }
            opt.set_lower_bounds(lb);
            opt.set_upper_bounds(ub);
        try
        {
            double final_cost;
            nlopt::result result = opt.optimize(q, final_cost);    
        }
        catch(std::exception &e)
        {
            std::cout << "nlopt failed: " << e.what() << std::endl;
        }

        for (size_t j = 0; j < cps_num_-2*p_order_; j++)
        {
            for (size_t i = 0; i < Dim_; i++)
            {
                control_points_(j+p_order_,i) = best_variable_[j*Dim_+i];
            } 
        }
        cout<< "optimize successfully~"<<endl;
            // cout << "iner:\n"<<control_points_<<endl;
            // cout<<"iter num :"<<iter_num_<<endl;
    }
 void bspline_optimizer::optimizesmooth()
    {
            iter_num_        = 0;
            min_cost_        = std::numeric_limits<double>::max();
            variable_num = (cps_num_-2*p_order_)*Dim_;
            nlopt::opt opt(nlopt::algorithm(nlopt::LD_LBFGS),variable_num);
            opt.set_min_objective(bspline_optimizer::costFunctionSmooth,this);
            opt.set_maxeval(200);
            opt.set_maxtime(0.02);
            opt.set_xtol_rel(1e-5);
            vector<double> lb(variable_num), ub(variable_num);
            vector<double> q(variable_num);

            for (size_t j = 0; j < cps_num_-2*p_order_; j++)
            {
               for (size_t i = 0; i < Dim_; i++)
                {
                    q[j*Dim_+i] = control_points_(j+p_order_,i);
                }
            }

            const double  bound = 5.0;
            for (size_t i = 0; i <variable_num; i++)
            {
                    lb[i]  = q[i]-bound;
                    ub[i] = q[i]+bound;      
            }
            opt.set_lower_bounds(lb);
            opt.set_upper_bounds(ub);
        try
        {
            double final_cost;
            nlopt::result result = opt.optimize(q, final_cost);    
        }
        catch(std::exception &e)
         {
            std::cout << "nlopt failed: " << e.what() << std::endl;
        }

          for (size_t j = 0; j < cps_num_-2*p_order_; j++)
            {
              for (size_t i = 0; i < Dim_; i++)
                {
                    control_points_(j+p_order_,i) = best_variable_[j*Dim_+i];
                } 
            }
            cout<< "optimize successfully~"<<endl;
    }

    void bspline_optimizer::optimize_withoutesdf()
    {
           double intial_lambda3 = lambda3_;
            lambda3_  = 0;
            /* initialize solver */
            iter_num_        = 0;
            min_cost_        = std::numeric_limits<double>::max();
            variable_num = (cps_num_-2*p_order_)*Dim_;
            nlopt::opt opt(nlopt::algorithm(nlopt::LD_LBFGS),variable_num);
            opt.set_min_objective(bspline_optimizer::costFunction,this);
            opt.set_maxeval(200);
            opt.set_maxtime(0.02);
            opt.set_xtol_rel(1e-5);
            vector<double> lb(variable_num), ub(variable_num);
            vector<double> q(variable_num);

            for (size_t j = 0; j < cps_num_-2*p_order_; j++)
            {
               for (size_t i = 0; i < Dim_; i++)
                {
                    q[j*Dim_+i] = control_points_(j+p_order_,i);
                }
            }

            const double  bound = 10.0;
            for (size_t i = 0; i <variable_num; i++)
            {
                    lb[i]  = q[i]-bound;
                    ub[i] = q[i]+bound;      
            }
            opt.set_lower_bounds(lb);
            opt.set_upper_bounds(ub);
        try
        {
            double final_cost;
            nlopt::result result = opt.optimize(q, final_cost);    
        }
        catch(std::exception &e)
         {
            std::cout << "nlopt failed: " << e.what() << std::endl;
        }

          for (size_t j = 0; j < cps_num_-2*p_order_; j++)
            {
              for (size_t i = 0; i < Dim_; i++)
                {
                    control_points_(j+p_order_,i) = best_variable_[j*Dim_+i];
                } 
            }
            lambda3_  = intial_lambda3;
    }



/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
   //只用于测试
    plan_manager::plan_manager(ros::NodeHandle &nh)
    {
        setParam(nh);
        TrajPlanning(nh);
    }
    plan_manager::~plan_manager() {}

    void plan_manager::setParam(ros::NodeHandle &nh)
    {
        last_time_ = ros::Time::now().toSec();
        last_time_ = ros::Time::now().toSec();
        nh.param("planning/traj_order", p_order_, 3);
        nh.param("planning/dimension", Dim_, -1);
        nh.param("planning/TrajSampleRate", TrajSampleRate, -1);
        nh.param("planning/max_vel", max_vel_, -1.0);
        nh.param("planning/max_acc", max_acc_, -1.0);
        nh.param("planning/goal_x", goal_x_, -1.0);
        nh.param("planning/goal_y", goal_y_, -1.0);
        nh.param("planning/lambda1",lambda1_,-1.0);
        nh.param("planning/lambda2",lambda2_,-1.0);
        nh.param("planning/lambda3",lambda3_,-1.0);
        nh.param("planning/lambda4",lambda4_,-1.0);
        // nh.param("planning/lambda5",lambda5_,-1.0);
        // nh.param("planning/lambda6",lambda6_,-1.0);
        nh.param("planning/frame",frame_,std::string("odom"));
        nh.param("planning/map_resolution",map_resolution_,-1.0);
        nh.param("planning/start_x",start_x_,-1.0);
        nh.param("planning/start_y",start_y_,-1.0);
        nh.param("planning/start_x",startPoint_x,-1.0);
        nh.param("planning/start_y",startPoint_y,-1.0);
        nh.param("planning/safe_distance",safe_distance_,-1.0);
        nh.param("planning/swarm_clearance",swarm_clearance_,-1.0);
        nh.param("planning/esdf_collision",esdf_collision,2.0);
        nh.param("planning/dist_p",dist_p,-1.0);
        nh.param("planning/drone_id", drone_id_,-1);
        // nh.param("planning/planning_horizen_time", planning_horizen_time_, -1.0);
        nh.param("planning/planning_horizon", planning_horizen_, -1.0);
        
        beta = max_vel_/dist_p;
        // cout<< "beta is "<<beta<<endl;

        have_odom_ = false;
        have_recv_pre_agent_ = false;
        lambda3_saved = lambda3_;
        get_path = false;
        // current_aim = Eigen::Vector2d::Zero();
        // current_vel = Eigen::Vector2d::Zero();
        // current_acc = Eigen::Vector2d::Zero();
        now_state_ = PLANNER_STATE::SEQUENTIAL_START;
        
        
        // beta = max_vel_/dist_p;
        // cout<< "beta is "<<beta<<endl;

        // lambda3_saved = lambda3_;
    }

    void plan_manager::TrajPlanning(ros::NodeHandle &nh)
    {
        //goal_suber = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, & plan_manager::uav_goal_subCallback, this);
        fsm_suber = nh.subscribe<std_msgs::Int64>("/flag_detect",1,&plan_manager::fsm_subCallback,this);

        //订阅地图
        map_suber = nh.subscribe<std_msgs::Float64MultiArray>("/ESDFmsgs",1,&plan_manager::esdf_map_subCallback,this);
        
        //订阅路径
        path_suber = nh.subscribe<nav_msgs::Path>("/astar_node/grid_twist",1, &plan_manager::astar_getCallback,this);

        //订阅起止点
        waypoint_suber = nh.subscribe<nav_msgs::Path>("/waypoint",1, &plan_manager::smooth_subCallback,this);

        //发布轨迹
        Traj_puber = nh.advertise<multi_bspline_opt::BsplineTraj>("/bspline_traj", 10);
        Time_puber = nh.advertise<std_msgs::Float64>("/back_time", 10);

        //可视化执行的轨迹   
        Traj_vis = nh.advertise<nav_msgs::Path>("/traj_vis", 10);
        Traj_vis1 = nh.advertise<nav_msgs::Path>("/traj_smooth", 10);

        //可视化地图
        Map_puber = nh.advertise<visualization_msgs::Marker>("/esdfmap_slice", 10);

        traj_smooth = nh.advertise<multi_bspline_opt::BsplineTraj>("bspline_smooth",10);

        col_check = nh.advertise<std_msgs::Bool>("/col_check",10);

        //同步飞机位置速度消息
        subscriber_vel = new message_filters::Subscriber<geometry_msgs::TwistStamped>(nh,"/mavros/local_position/velocity_local_orb",10);
        subscriber_pos = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh,"/mavros/local_position/pose_orb",10);
        subscriber_acc = new message_filters::Subscriber<sensor_msgs::Imu>(nh,"/mavros/imu/data",10);
        sync = new message_filters::Synchronizer<syncPolicy>(syncPolicy(10), *subscriber_vel, *subscriber_pos, *subscriber_acc);  
        sync->registerCallback(boost::bind(&plan_manager::current_state_callback,this, _1, _2, _3));

//获取当前无人机位置
        odom_suber = nh.subscribe("odom",10, &plan_manager::OdomCallback, this);
        //获取当前aim
        // fullaim_suber = nh.subscribe<mavros_msgs::PositionTarget>("/mavbs/setpoint_raw/local",1,&plan_manager::fullaim_callback,this);
        arrived_suber = nh.subscribe<std_msgs::Int64>("/astar_node/target_arrived",10,&plan_manager::arrive_callback,this);

      //多机
        broadcast_bspline_sub_ = nh.subscribe("/broadcast_bspline", 100, &plan_manager::BroadcastBsplineCallback, this, ros::TransportHints().tcpNoDelay());
        //发布自己的轨迹
        broadcast_bspline_pub_ = nh.advertise<multi_bspline_opt::SendTraj>("/broadcast_bspline", 10);   //发布大广播
        //用于启动
        if (drone_id_  >= 1 )
        {
           //订阅优先级更高一级的无人机的轨迹   
           string sub_topic_name = string("/uav") + std::to_string(drone_id_ - 1) + string("_planning_swarm_trajs");
           swarm_trajs_sub_ = nh.subscribe(sub_topic_name.c_str(), 10, &plan_manager::swarmTrajsCallback, this, ros::TransportHints().tcpNoDelay());
        }
//发布swarm轨迹(绝对里程计坐标)
        string pub_topic_name = string("/uav") + std::to_string(drone_id_) + string("_planning_swarm_trajs");
        swarm_trajs_pub_ = nh.advertise<multi_bspline_opt::MultiBsplines>(pub_topic_name.c_str(), 10);

        //   traj_timer_ = nh.createTimer(ros::Duration(0.01), &plan_manager::stateFSMCallback, this);
    }
void plan_manager::arrive_callback(const std_msgs::Int64::ConstPtr & msg)
{
    if(msg->data>0)
    {
        lambda3_ = lambda3_saved;
    }
    // cout<<lambda3_<<endl;
}
void plan_manager::fsm_subCallback(const std_msgs::Int64::ConstPtr & msg)
{
    if(msg->data == 3)
        enable_flag = true;
    else 
        enable_flag = false;
}

 void plan_manager::StateChange( PLANNER_STATE new_state, string pos_call)
 {
    //如果状态没有变
     if (new_state == now_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[3] = { "FLYING", "SEQUENTIAL_START"};
    int pre_s = int(now_state_);
    now_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(now_state_)] << endl;
 }
  void plan_manager::stateFSMCallback()
  {
    // traj_timer_.stop(); // To avoid blockage
    // cout<<"--------------drone_id_:"<<drone_id_<<endl;
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      if (!have_odom_)
      {
        cout << "no odom." << endl;
        return;
      }
        
      fsm_num = 0;
    }
    switch (now_state_)
    {
    case SEQUENTIAL_START:
    {   
        if (drone_id_ <= 0 || (drone_id_ >= 1 && have_recv_pre_agent_))
      {
        if (have_odom_ && get_path)
        {
          bool success = astar_subCallback(astar_path_); 
          if (success)
          {
            // ROS_INFO(" generate the  trajectory Success!!!");
             PublishSwarm(true);
             StateChange(FLYING, "my planner");
           
          }
          else
          {
            // ROS_ERROR("Failed to generate the  trajectory!!!");
          }
        }
        else
        {
          // ROS_ERROR("No odom or not recieved A_star! have_odom_=%d, get_path=%d", have_odom_, get_path);
        }
      }
      break;
    }

    case FLYING: // for swarm
    {
      if (drone_id_ <= 0 || (drone_id_ >= 1 && have_recv_pre_agent_))
      {
        if (have_odom_ && get_path)
        {
          bool success = astar_subCallback(astar_path_); 
          if (success)
          { 
            PublishSwarm(false);
            // ROS_INFO(" generate the  trajectory Success!!!");
            StateChange(FLYING, "my planner");
           
          }
          else
          {
            // ROS_ERROR("Failed to generate the  trajectory!!!");
          }
        }
        else
        {
          // ROS_ERROR("No odom or not recieved A_star! have_odom_=%d, get_path=%d", have_odom_, get_path);
        }
      }
      break;
    }

  } 
    //  traj_timer_.start();
  }
// void plan_manager::fullaim_callback(const mavros_msgs::PositionTarget::ConstPtr & aim_msg)
// {
//     current_aim[0] = aim_msg->position.x;
//     current_aim[1] = aim_msg->position.y;
//     current_vel[0] = aim_msg->velocity.x;
//     current_vel[1] = aim_msg->velocity.y;
//     current_acc[0] = aim_msg->acceleration_or_force.x;
//     current_acc[1] = aim_msg->acceleration_or_force.y;
//     current_seq = (int)aim_msg->yaw;
//     // cout << "当前位置：\n" << current_aim <<endl;
    
//     back_time_= ros::Time::now().toSec();
// }

bool  plan_manager::astar_subCallback(const std::vector<Eigen::Vector2d> &astar_path_)
    {
        //读取首末位置
        Eigen::Vector2d start_point,end_point;
        start_point = *astar_path_.begin();
        end_point = *(astar_path_.end()-1);
        initial_state.resize(3,2);
        terminal_state.resize(3,2);
        std_msgs::Float64 msg_time;
        msg_time.data = back_time_;
        Time_puber.publish(msg_time);    
        initial_state << current_aim(0), current_aim(1),
                         current_vel(0), current_vel(1),
                         current_acc(0), current_acc(1);
        terminal_state << end_point(0), end_point(1),
		                    0.0, 0.0,
	                        0.0, 0.0;
        double now_time_  = ros::Time::now().toSec() ;
        // double duration_time;
        double delta_time = now_time_ - last_time_;//|| last_endpoint != end_point
        if( first_rifine == true || delta_time > 0.2 || checkTrajCollision() == true )// 
        {
            last_time_ = now_time_;
            first_rifine = false;
            last_endpoint = end_point;
            //初始化优化类和 b样条轨迹类
            opt.reset(new bspline_optimizer(astar_path_,Dim_,p_order_));
            u.reset(new UniformBspline(p_order_,opt->cps_num_,beta,Dim_,initial_state,terminal_state));
            UniformBspline spline = *u;
            opt->setEsdfMap(esdf_map_) ;
            opt->setSwarmTrajs(&swarm_trajs_buf_);
            opt->setOptParam(lambda1_,lambda2_,lambda3_,lambda4_,safe_distance_,swarm_clearance_);
            opt->setMapParam(origin_x_,origin_y_,map_resolution_,start_x_,start_y_);
            opt->setVelAcc(max_vel_,max_acc_);
            opt->setSplineParam(spline);
            opt->optimize();
            // 计算轨迹
            // cout <<"\033[46m--------------------grid_path - control_points_--------------------"<<endl;
            int i = 0;
            for(auto ptr : astar_path_)
            {Eigen::Vector2d xxx(opt->control_points_(3+i,0),opt->control_points_(3+i,1));
            // cout << Eigen::Vector2d(ptr - xxx).norm() <<endl;
            i++;
            }
            // cout <<"----------------------------------------\033[0m"<<endl;
            auto old_ptr = (*astar_path_.begin());
            // cout <<"\033[45m--------------------grid_path--------------------"<<endl;
            for(auto ptr : astar_path_)
            {
            // cout << ptr <<endl;
            // cout << "- - -"<<endl;
            // cout << Eigen::Vector2d(ptr - old_ptr).norm() <<endl;
            // cout << "- - -"<<endl;
            old_ptr = ptr;
            }
            // cout <<"----------------------------------------\033[0m"<<endl;
            // cout <<"\033[45m--------------------control_points_--------------------"<<endl;
            i = 0;
            for(auto ptr : astar_path_)
            {Eigen::Vector2d xxx(opt->control_points_(3+i,0),opt->control_points_(3+i,1));
            // cout << xxx <<endl;
            // cout << "- - -"<<endl;
            i++;
            }
            // cout <<"----------------------------------------\033[0m"<<endl;
            u->setControlPoints(opt->control_points_);
            u->getT(TrajSampleRate);
            UniformBspline p = *u;
            UniformBspline v = p.getDerivative();
            UniformBspline a = v.getDerivative();
            double duration_time = p.getAvailableTrange();

            //生成轨迹
            geometry_msgs::PoseStamped tmp_p,tmp_v,tmp_a;
            geometry_msgs::Point tmp_p_pub;
            geometry_msgs::PoseStamped tmp_vis;
            // std::vector<Eigen::Vector2d> 
            p_ = p.getTrajectory(p.time_);
            v_ = v.getTrajectory(p.time_);
            a_ = a.getTrajectory(p.time_);

            traj.position.clear();
            traj.velocity.clear();
            traj.acceleration.clear();
            traj_vis.poses.clear();      

            //发布给其它无人机
            traj_pub.control_pts.clear();
            traj_pub.knots.clear();
           for (size_t i = 0; i < u->getKnot().size(); i++)
            {           
              double tmp_knot;
              tmp_knot =  u->getKnot()(i);
              traj_pub.knots.push_back(tmp_knot);
            }            
            traj_pub.drone_id = drone_id_;
            traj_pub.cps_num_ = opt->cps_num_;
            traj_pub.start_pos_x = current_aim(0);
            traj_pub.start_pos_y = current_aim(1);
            traj_pub.start_vel_x = current_vel(0);
            traj_pub.start_vel_y = current_vel(1);
            traj_pub.start_acc_x = current_acc(0);
            traj_pub.start_acc_y = current_acc(1);
            traj_pub.end_pos_x = end_point(0);
            traj_pub.end_pos_y = end_point(1);               
            geometry_msgs::Point tmp_control;
            for (size_t i = 0; i < opt->control_points_.rows(); i++)
            {
              tmp_control.x =opt->control_points_(i,0);
              tmp_control.y = opt->control_points_(i,1);
              tmp_control.z =  0;
              traj_pub.control_pts.push_back(tmp_control);
            }           
             for (size_t i = 0; i < p_.rows(); i++)
            {
                int count = i;
                tmp_p.header.seq = count;
                tmp_v.header.seq = count;
                tmp_a.header.seq = count;
                // tmp_p_pub.x   = p_(count,0);tmp_p_pub.y   = p_(count,1);tmp_p_pub.z   = 0;
                tmp_p.pose.position.x   = p_(count,0);
                tmp_p.pose.position.y   = p_(count,1);
                tmp_p.pose.position.z   = 0;
                tmp_v.pose.position.x   = v_(count,0); tmp_v.pose.position.y  = v_(count,1);tmp_v.pose.position.z   = 0;
                tmp_a.pose.position.x   = a_(count,0); tmp_a.pose.position.y  = a_(count,1); tmp_a.pose.position.z  = 0;
                tmp_vis.pose.position.x = p_(count,0);tmp_vis.pose.position.y = p_(count,1);tmp_vis.pose.position.z = 0.1;
                traj.position.push_back(tmp_p) ;
                traj.velocity.push_back(tmp_v) ;
                traj.acceleration.push_back(tmp_a);
                traj_vis.poses.push_back(tmp_vis);
                traj_vis.header.frame_id = frame_;
            }

            Traj_vis.publish(traj_vis);
            Traj_vis1.publish(traj_vis1);
            // traj_pub.drone_id = drone_id_;

            // traj_pub.start_time = ros::Time::now();
            // traj_pub.duration  = duration_time;
            traj.current_seq = current_seq;

            traj_pub.order = p_order_;
            traj_pub.start_time = ros::Time::now();
            traj_pub.traj_id =  drone_id_;
            // traj_pub.cps_num_ = cps_num_
            Traj_puber.publish(traj);
            return true;
        }
            
    }

void plan_manager::PublishSwarm(bool startup_pub)
{
                //发布swarm为earth坐标
         if (startup_pub)
             {
                multi_bspline_msgs_buf_.drone_id_from = drone_id_; // zx-todo
              if ((int)multi_bspline_msgs_buf_.traj.size() == drone_id_ + 1)
                {
                    multi_bspline_msgs_buf_.traj.back() = traj_pub;
                 }
              else if ((int)multi_bspline_msgs_buf_.traj.size() == drone_id_)
              {
                    multi_bspline_msgs_buf_.traj.push_back(traj_pub);
             }
            swarm_trajs_pub_.publish(multi_bspline_msgs_buf_);
             }
            broadcast_bspline_pub_.publish(traj_pub);
}    
void plan_manager::current_state_callback(const geometry_msgs::TwistStampedConstPtr & vel_msg,
                                          const geometry_msgs::PoseStampedConstPtr &pos_msg,
                                          const sensor_msgs::ImuConstPtr &imu_msg)
    {
        // current_pos<< pos_msg->pose.position.x,pos_msg->pose.position.y;
        // current_vel<< vel_msg->twist.linear.x,vel_msg->twist.linear.y;
        // current_acc<< imu_msg->linear_acceleration.x,-imu_msg->linear_acceleration.y;// BUG!!
    }

void plan_manager::OdomCallback(const nav_msgs::Odometry &pos_msg)
{
            have_odom_ = true;
        drone_pos_world(0) = pos_msg.pose.pose.position.x;
        drone_pos_world(1) = pos_msg.pose.pose.position.y;
        // drone_pos_world(2) = pos_msg->pose.position.z;
    //odom_acc_ = estimateAcc( msg );

        // odom_orient_.w() = pos_msg.pose.pose.orientation.w;
        // odom_orient_.x() =  pos_msg.pose.pose.orientation.x;
        // odom_orient_.y() =  pos_msg.pose.pose.orientation.y;
        // odom_orient_.z() =  pos_msg.pose.pose.orientation.z;
    

}
    void plan_manager::uav_goal_subCallback(const geometry_msgs::PoseStampedConstPtr &goal_msg)
    {
        terminal_state(0,0) = goal_msg->pose.position.x;
        terminal_state(0,1) = goal_msg->pose.position.y; 
    }

    void plan_manager::esdf_map_subCallback(const std_msgs::Float64MultiArrayConstPtr &map_msg)
    {
        // cout<< "get grid map"<<endl;
        get_map = true;
        map_size_x = map_msg->data[0];
        map_size_y = map_msg->data[1];
        map_size = map_size_x*map_size_y;
        grid_map_.setZero(map_size_x,map_size_y);//grid map 初始化全0
        esdf_map_=100* Eigen::MatrixXd::Ones(map_size_x,map_size_y);//esdf map 初始化全100
        double *src,*dst;
        src=(double*)malloc(map_size*sizeof(double));
        dst=(double*)malloc(map_size*sizeof(double));

        for (size_t i = 0; i < map_size_x; i++)
        {
            for (size_t j = 0; j < map_size_y; j++)
            {
                grid_map_(map_size_x-j-1,i) = map_msg->data[i*map_size_x+j+2];
            }
        }

        for (size_t i = 0; i < map_size_x; i++)
        {
            for (size_t j = 0; j < map_size_y; j++)
            {
                //把grid_map里的数据送入src指针
                *(src+i*esdf_map_.rows()+j) = grid_map_(i,j);
                
            }
        }

        computeEDT(dst,src,map_size_x,map_size_y);

        for (size_t i = 0; i < map_size_x; i++)
        {
            if( (int)sqrt(*(dst+i*esdf_map_.rows()))<-1000)break;
            else
            {
            for (size_t j = 0; j < map_size_y; j++)
            {
                esdf_map_(i,j) = (int)sqrt(*(dst+i*esdf_map_.rows()+j));
            }
            }
        }      
        map_slice_output(esdf_map_);
        // map_slice_output(grid_map_);
        free(dst);
        free(src);
        dst = NULL;
        src = NULL;

    }



    void plan_manager::astar_getCallback(const nav_msgs::PathConstPtr &path)
    {
        if(!get_map) return;
        //get_map = false;
        astar_path_.clear();
        get_path = true;
        //读取Astar
        Eigen::Vector2d tmp_point;
        double delta_t = 0.1;
        current_aim = Eigen::Vector2d::Zero();
        current_vel = Eigen::Vector2d::Zero();
        current_acc = Eigen::Vector2d::Zero();
        current_vel << path->poses[0].pose.position.x,path->poses[0].pose.position.y;
        current_seq = path->poses[0].pose.position.z;
        current_acc << path->poses[1].pose.position.x,path->poses[1].pose.position.y;
        current_aim << path->poses[2].pose.position.x,path->poses[2].pose.position.y;
        for (size_t i = 2; i < path->poses.size(); i++)
        {
            tmp_point<< path->poses[i].pose.position.x,path->poses[i].pose.position.y;
            //A star路径是正的
            astar_path_.push_back(tmp_point);
        }
            //读取首末位置
         static int fsm_num = 0;
         fsm_num++;
         if (fsm_num == 100)
        {
        if (!have_odom_)
       {
        cout << "no odom." << endl;
        return;
      }
        
      fsm_num = 0;
    }    
     stateFSMCallback();
            
    }

void plan_manager::BroadcastBsplineCallback(const multi_bspline_opt::SendTraj::ConstPtr &msg)
{
    size_t id = msg->drone_id;
    //如果是自己轨迹，返回
    if ((int)id == drone_id_)
      return;
    //如果轨迹和现在时间差太大，返回
    // cout<<"start_time_msg:"<<msg->start_time.toSec()<<endl;
    if (abs((ros::Time::now() - msg->start_time).toSec()) > 2.0)
    {
      ROS_ERROR("Time difference is too large! Local - Remote Agent %d = %fs",
     msg->drone_id, (ros::Time::now() - msg->start_time).toSec());
      return;
    }    

//初始化
    if (swarm_trajs_buf_.size() <= id)
    {
      for (size_t i = swarm_trajs_buf_.size(); i <= id; i++)
      {
        OneTrajDataOfSwarm blank;
        blank.drone_id = -1;
        swarm_trajs_buf_.push_back(blank);
      }
    }
//获得odom坐标系坐标
    // size_t id = msg->drone_id;
    
    //change to map
    Eigen::Vector2d cp0(msg->control_pts[0+msg->order].x, msg->control_pts[0+msg->order].y);
    Eigen::Vector2d cp1(msg->control_pts[1+msg->order].x, msg->control_pts[1+msg->order].y);
    Eigen::Vector2d cp2(msg->control_pts[2+msg->order].x, msg->control_pts[2+msg->order].y);
    Eigen::Vector2d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
    //如果无人机和当前无人机距离太远，则忽略
    // cout<<"swarm_start_pt:"<<swarm_start_pt<<"drone_pos_world:"<<drone_pos_world<<endl;
    // cout<<"distance:"<<(swarm_start_pt - drone_pos_world).norm() <<endl;
    // cout<<"safe："<<planning_horizen_<<endl;
    if ((swarm_start_pt - drone_pos_world).norm() > planning_horizen_ *4.0f/3.0f )
    {
       swarm_trajs_buf_[id].drone_id = -1;
       ROS_ERROR("TOO FAR!");
       return; 
    }
    //    return;
    //swarm_trajs_buf_里存储odom坐标系坐标
    swarm_trajs_buf_[id].drone_id = id;
    Eigen::MatrixXd pos_pts( msg->control_pts.size(),2);
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t j = 0; j < msg->knots.size(); ++j)
    {
      knots(j) = msg->knots[j];
    }
    for (size_t j = 0; j < msg->control_pts.size(); ++j)
    {
      pos_pts( j,0) = msg->control_pts[j].x;
      pos_pts(j,1) = msg->control_pts[j].y;
    }

//将轨迹存储
    // swarm_trajs_buf_[id].drone_id = id;
    
    if (msg->order % 2)
    {
      double cutback = (double)msg->order / 2 + 1.5;
      swarm_trajs_buf_[id].duration_ = msg->knots[msg->knots.size() - ceil(cutback)];
      
    }
    else
    {
      double cutback = (double)msg->order / 2 + 1.5;
      swarm_trajs_buf_[id].duration_ = (msg->knots[msg->knots.size() - floor(cutback)] + msg->knots[msg->knots.size() - ceil(cutback)]) / 2;
    }
    // cout<<"swarm_trajs_buf_[id].duration_"<<swarm_trajs_buf_[id].duration_<<endl;
   cout<<"error flag777"<<endl; 
   Eigen::MatrixXd init;
    Eigen::MatrixXd end;
    init.resize(3,2);
    end.resize(3,2);
    init<<msg->start_pos_x, msg->start_pos_y,
               msg->start_vel_x, msg->start_vel_y,
               msg->start_acc_x, msg->start_acc_y;
    end<<msg->end_pos_x, msg->end_pos_y,
               0.0, 0.0,
               0.0,   0.0;
    cout<<"error flag6661"<<endl;    
    UniformBspline pos_traj(p_order_,msg->cps_num_,beta, Dim_, init, end);
    pos_traj.setControlPoints(pos_pts);
    pos_traj.getT(TrajSampleRate);
    swarm_trajs_buf_[id].position_traj_ = pos_traj;
    // pos_traj.setKnot(knots);

    // swarm_trajs_buf_[id].position_traj_ = pos_traj;
        Eigen::Vector2d start(msg->start_pos_x, msg->start_pos_y);
       swarm_trajs_buf_[id].start_pos_ = start;

       swarm_trajs_buf_[id].start_time_ = msg->start_time;

}
/*****************************************
 * smooth_path
 * 输入 :nav_msgs::Path
 * 输出:同astar
 * 
 * *************************************/
//接受轨迹
void plan_manager::swarmTrajsCallback(const multi_bspline_opt::MultiBsplinesPtr &msg)
{
    ROS_ERROR("have_rec, %d", have_recv_pre_agent_);
    //获取轨迹
        multi_bspline_msgs_buf_.traj.clear();
        multi_bspline_msgs_buf_ = *msg;

         if (!have_odom_)
          {
            //  ROS_ERROR("swarmTrajsCallback(): no odom!, return.");
              return;
          }
     cout<<"traj"<<(int)msg->traj.size()<<"id from"<< msg->drone_id_from<<endl; 

    if ((int)msg->traj.size() != msg->drone_id_from + 1) // drone_id must start from 0
    {
      // ROS_ERROR("Wrong trajectory size! msg->traj.size()=%d, msg->drone_id_from+1=%d", (int)msg->traj.size(), msg->drone_id_from + 1);
      return;
    }
   
    // if (msg->traj[0].order != 3) // only support B-spline order equals 3.
    // {
    //   ROS_ERROR("Only support B-spline order equals 3.");
    //   return;
    // }

    swarm_trajs_buf_.clear();
    swarm_trajs_buf_.resize(msg->traj.size());
    
    for (size_t i = 0; i < msg->traj.size(); i++)
    {
    Eigen::Vector2d cp0(msg->traj[i].control_pts[0+msg->traj[i].order].x, msg->traj[i].control_pts[0+msg->traj[i].order].y);
    Eigen::Vector2d cp1(msg->traj[i].control_pts[1+msg->traj[i].order].x, msg->traj[i].control_pts[1+msg->traj[i].order].y);
    Eigen::Vector2d cp2(msg->traj[i].control_pts[2+msg->traj[i].order].x, msg->traj[i].control_pts[2+msg->traj[i].order].y);
    Eigen::Vector2d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
    //如果无人机和当前无人机距离太远，则忽略
    if ((swarm_start_pt - drone_pos_world).norm() > planning_horizen_ * 4.0f / 3.0f)
    {
       swarm_trajs_buf_[i].drone_id = -1;
      continue; 
    } 

    swarm_trajs_buf_[i].drone_id = i;
        // cout<<"error flag22"<<endl;
    Eigen::MatrixXd pos_pts( msg->traj[i].control_pts.size(),2);
    Eigen::VectorXd knots(msg->traj[i].knots.size());
    // cout<<"error flag23"<<endl;
      for (size_t j = 0; j < msg->traj[i].control_pts.size(); ++j)
      {
    
        pos_pts( j,0) = msg->traj[i].control_pts[j].x;
        pos_pts(j,1) = msg->traj[i].control_pts[j].y;

      } 

   if (msg->traj[i].order % 2)
    {
      double cutback = (double)msg->traj[i].order / 2 + 1.5;
      swarm_trajs_buf_[i].duration_ = msg->traj[i].knots[msg->traj[i].knots.size() - ceil(cutback)];
    }
    else
    {
      double cutback = (double)msg->traj[i].order / 2 + 1.5;
      swarm_trajs_buf_[i].duration_ = (msg->traj[i].knots[msg->traj[i].knots.size() - floor(cutback)] + msg->traj[i].knots[msg->traj[i].knots.size() - ceil(cutback)]) / 2;
    } 
    // cout<<"error flag27"<<endl;
    Eigen::MatrixXd init;
    Eigen::MatrixXd end;
    init.resize(3,2);
    end.resize(3,2);
    //    cout<<"error flag666"<<endl;    
  
    //  double start_x = msg->traj[i].start_pos_x;    
    //   cout<<"start1:"<< msg->traj[i].start_vel_x<<"  "<<"end"<<msg->traj[i].start_vel_y<<endl;
    //     cout<<"acc1:"<< msg->traj[i].start_acc_x<<"  "<<"acc end"<< msg->traj[i].start_acc_y<<endl;
       init<<msg->traj[i].start_pos_x, msg->traj[i].start_pos_y,
               msg->traj[i].start_vel_x, msg->traj[i].start_vel_y,
               msg->traj[i].start_acc_x,   msg->traj[i].start_acc_y;
        cout<<"error flag28"<<endl;
        end<<msg->traj[i].end_pos_x, msg->traj[i].end_pos_y,
               0.0, 0.0,
               0.0,   0.0;

    UniformBspline pos_traj(p_order_,msg->traj[i].cps_num_,beta, Dim_, init, end);
    pos_traj.setControlPoints(pos_pts);
    pos_traj.getT(TrajSampleRate);
    swarm_trajs_buf_[i].position_traj_ = pos_traj;
    // pos_traj.setKnot(knots);

    // swarm_trajs_buf_[id].position_traj_ = pos_traj;
        Eigen::Vector2d start(msg->traj[i].start_pos_x, msg->traj[i].start_pos_y);
       swarm_trajs_buf_[i].start_pos_ = start;

       swarm_trajs_buf_[i].start_time_ = msg->traj[i].start_time;

    }
   
    have_recv_pre_agent_ = true;
    

}
void plan_manager::smooth_subCallback(const nav_msgs::PathConstPtr &msg)
{

}

void plan_manager::map_slice_output(const Eigen::MatrixXd &esdf_matrix)
{
    visualization_msgs::Marker marker_result;
    marker_result.header.frame_id = "/world";
    marker_result.type = visualization_msgs::Marker::POINTS;
    marker_result.action = visualization_msgs::Marker::MODIFY;
    marker_result.scale.x = 0.1;
    marker_result.scale.y = 0.1;
    marker_result.scale.z = 0.1;
    marker_result.pose.orientation.x = 0;
    marker_result.pose.orientation.y = 0;
    marker_result.pose.orientation.z = 0;
    marker_result.pose.orientation.w = 1;

    /* 遍历矩阵的所有元素 */
    for (int i = 0; i < esdf_matrix.rows(); i++)
    {
        for (int j = 0; j < esdf_matrix.cols(); j++)
        {
            double h = esdf_matrix(i, j);
            double max_dist = 20.0;
            if (h < -15.0 || h > 15.0) continue;
            /* 计算当前栅格中心的真实坐标 */
            double vox_pos_x, vox_pos_y;
            vox_pos_x = (j+0.5)*0.1 + (startPoint_x-0.05);
            vox_pos_y = (startPoint_y+0.05) - (i+0.5)*0.1;
            geometry_msgs::Point pt;
            pt.x = vox_pos_x;
            pt.y = vox_pos_y;
            pt.z = -0.5;
            marker_result.points.push_back(pt);

            /* 计算色彩 */
            std_msgs::ColorRGBA color;
            color.a = 1;
            std::vector<float> color_result;
            color_result = calculate_color(h, max_dist, -max_dist, R, G, B);
            color.r = color_result[0];
            color.g = color_result[1];
            color.b = color_result[2];
            marker_result.colors.push_back(color);
        }
    }
    Map_puber.publish(marker_result);
    }
/*************************************************
 * 计算渐变色
 *  输入：
 *      ESDF值；
 *      最大距离阈值；
 *      最小距离阈值；
 *      RGB色表；
 *  输出：
 *      R、G、B值；
 *************************************************/
std::vector<float>  plan_manager::calculate_color(double esdf_value, double max_dist, double min_dist, std::vector<int> R_values, std::vector<int> G_values, std::vector<int> B_values)
{
    std::vector<float> color_result;

    /* 分段上色 */
    int colors_num = R_values.size();
    double dist_seg = (max_dist - min_dist) / colors_num;
    if (esdf_value > max_dist) esdf_value = max_dist;
    if (esdf_value < min_dist) esdf_value = min_dist;
    int seg_num = floor( (esdf_value - min_dist) / dist_seg );
    color_result.push_back((float)R_values[seg_num]/255.0);
    color_result.push_back((float)G_values[seg_num]/255.0);
    color_result.push_back((float)B_values[seg_num]/255.0);

    return color_result;
}

bool plan_manager::checkTrajCollision()
{   
    traj_state state;//判断算出来的轨迹是否安全
    state = SAFE;
    Eigen::Vector2i tmp_index;//
    for (size_t i = 0; i < p_.rows(); i++)
    {
        tmp_index = posToIndex(p_.row(i));
        if(esdf_map_(tmp_index(0),tmp_index(1))<=esdf_collision)
        {
            cout << "colision!"<<endl;
            state = COLLIDE;
            break;
        }
    }
    if(state == COLLIDE)
    {
        return true;//会撞
    } 
    else
        return false;//不会撞
}
Eigen::MatrixXd plan_manager::getSmoothTraj(const geometry_msgs::PoseStamped &start,
                                                                                                 const geometry_msgs::PoseStamped &end)
{
    //目前该函数仅考虑二维平面的运动，三维运动将会尽快迭代
    Eigen::MatrixXd traj;
    initial_state.resize(3,2);
    terminal_state.resize(3,2);
    initial_state <<start.pose.position.x, start.pose.position.y,
		                    1.0, 0.0,
	                        3.0, 0.0;
    terminal_state<< end.pose.position.x, end.pose.position.y,
		                    0.0, 0.0,
	                        0.0, 0.0;
    double dist = sqrt(pow(end.pose.position.x-start.pose.position.x,2)+
                                pow(end.pose.position.y-start.pose.position.y,2));
        
    opt.reset(new bspline_optimizer(Dim_,p_order_,dist));//只考虑smooth的构造
    u.reset(new UniformBspline(p_order_,opt->cps_num_,beta,Dim_,initial_state,terminal_state));
    opt->setSmoothParam(lambda1_,lambda2_,max_vel_,max_acc_);
    opt->initialControlPoints(*u);
    opt->optimizesmooth();            //计算轨迹

    u->setControlPoints(opt->control_points_);
    u->getT(TrajSampleRate);
    UniformBspline p = *u;
    UniformBspline v = p.getDerivative();
    UniformBspline a = v.getDerivative();
    p_ = p.getTrajectory(p.time_);
    v_ = v.getTrajectory(p.time_);
    a_ = a.getTrajectory(p.time_);
    traj.resize(p_.rows(),p_.cols()*3);
    //traj : px py vx vy ax ay
    for (size_t i = 0; i < traj.rows(); i++)
    {    
        traj(i,0)= p_(i,0);
        traj(i,1)= p_(i,1); 
        traj(i,2)= v_(i,0);
        traj(i,3)= v_(i,1);
        traj(i,4)= a_(i,0);
        traj(i,5)= a_(i,1);
    }   
    return traj;
}
}
