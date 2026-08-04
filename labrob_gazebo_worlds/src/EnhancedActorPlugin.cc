#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>
#include <random>
#include <chrono>
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <cstdlib>

namespace gazebo
{
  class EnhancedActorPlugin : public ModelPlugin
  {
    public:
      enum BehaviorType {
        STATIC,           // Standing still
        STATIC_PAIR,      // Standing still with a partner offset
        WALKING_SOLO,     // Walking alone with random waypoints
        WALKING_PAIR      // Walking with a partner (needs partner name)
      };
    
    private: 
      physics::ActorPtr actor;
      event::ConnectionPtr updateConnection;
      
      // Behavior configuration
      BehaviorType behavior;
      std::string partnerName;  // For WALKING_PAIR
      physics::ActorPtr partner; // Pointer to partner actor
      std::string pairKey;
      bool isPairLeader;
      double pairLateralOffset;
      double formationDistance;
      ignition::math::Vector3d lastPartnerDir;
      bool hasLastPartnerDir;
      
      // Movement parameters
      std::vector<ignition::math::Vector3d> waypoints;
      unsigned int targetIdx;
      double velocity;
      double lastTime;
      ignition::math::Vector3d currentPos;
      double initialYaw;
      bool staticPoseInitialized;
      
      // Pair configuration helper
      void ConfigurePairDefaults()
      {
        this->pairKey.clear();
        this->isPairLeader = true;
        this->pairLateralOffset = 0.0;
        this->formationDistance = 0.8;
      }

      static std::uint32_t HashString(const std::string &value)
      {
        std::uint32_t hash = 2166136261u;
        for (const unsigned char c : value)
        {
          hash ^= c;
          hash *= 16777619u;
        }
        if (hash == 0u)
          hash = 1u;
        return hash;
      }

      static std::uint32_t GetEnvScenarioSeed()
      {
        static bool initialized = false;
        static std::uint32_t envSeed = 0u;
        if (!initialized)
        {
          const char *env = std::getenv("CROWD_SIM_SEED");
          if (env)
          {
            envSeed = static_cast<std::uint32_t>(std::strtoul(env, nullptr, 10));
          }
          initialized = true;
        }
        return envSeed;
      }

      std::uint32_t ResolveSeed(sdf::ElementPtr sdf, bool deterministic, std::uint32_t deterministicSeed)
      {
        std::uint32_t userSeed = sdf->Get<unsigned int>("seed", 0u).first;
        std::uint32_t scenarioSeed = sdf->Get<unsigned int>("scenario_seed", 0u).first;
        if (scenarioSeed == 0u)
          scenarioSeed = GetEnvScenarioSeed();

        std::uint32_t baseSeed = 0u;
        if (userSeed != 0u)
          baseSeed = userSeed;
        else if (deterministic)
          baseSeed = deterministicSeed;
        else
          baseSeed = HashString(this->actor->GetName());

        if (scenarioSeed != 0u)
          baseSeed ^= scenarioSeed;

        if (baseSeed == 0u)
          baseSeed = 1u;

        return baseSeed;
      }

      std::uint32_t MixWithScenarioSeeds(sdf::ElementPtr sdf, std::uint32_t baseSeed)
      {
        std::uint32_t userSeed = sdf->Get<unsigned int>("seed", 0u).first;
        std::uint32_t scenarioSeed = sdf->Get<unsigned int>("scenario_seed", 0u).first;
        if (scenarioSeed == 0u)
          scenarioSeed = GetEnvScenarioSeed();

        if (userSeed != 0u)
          baseSeed ^= userSeed;

        if (scenarioSeed != 0u)
          baseSeed ^= scenarioSeed;

        if (baseSeed == 0u)
          baseSeed = 1u;

        return baseSeed;
      }

      double SampleYawFromSeed(std::uint32_t seed)
      {
        const double pi = 3.14159265358979323846;
        std::mt19937 rng(seed);
        std::uniform_real_distribution<double> dist(-pi, pi);
        return dist(rng);
      }

      ignition::math::Vector3d SampleStaticXY(sdf::ElementPtr sdf,
                                              const std::string &seedLabel)
      {
        ignition::math::Vector3d defaultPos = this->actor->WorldPose().Pos();
        double minX = sdf->Get<double>("min_x", defaultPos.X()).first;
        double maxX = sdf->Get<double>("max_x", defaultPos.X()).first;
        double minY = sdf->Get<double>("min_y", defaultPos.Y()).first;
        double maxY = sdf->Get<double>("max_y", defaultPos.Y()).first;

        if (minX > maxX)
          std::swap(minX, maxX);
        if (minY > maxY)
          std::swap(minY, maxY);

        std::uint32_t seed = HashString(seedLabel);
        seed = this->MixWithScenarioSeeds(sdf, seed);
        std::mt19937 rng(seed);
        std::uniform_real_distribution<double> rx(minX, maxX);
        std::uniform_real_distribution<double> ry(minY, maxY);

        ignition::math::Vector3d sampled(rx(rng), ry(rng), defaultPos.Z());
        return sampled;
      }

      void ApplyStaticXY(const ignition::math::Vector3d &position)
      {
        ignition::math::Pose3d pose = this->actor->WorldPose();
        ignition::math::Vector3d updated = pose.Pos();
        updated.X(position.X());
        updated.Y(position.Y());
        pose.Set(updated, pose.Rot());
        this->actor->SetWorldPose(pose, false, false);
        this->currentPos = updated;
      }

      void ApplyStaticYaw(double yaw)
      {
        ignition::math::Pose3d pose = this->actor->WorldPose();
        ignition::math::Quaterniond rot(1.5707, 0, yaw);
        pose.Set(pose.Pos(), rot);
        this->actor->SetWorldPose(pose, false, false);
      }

      void UpdatePairFollower(const common::UpdateInfo &info)
      {
        if (!this->partner)
        {
          this->partner = boost::dynamic_pointer_cast<physics::Actor>(
            this->actor->GetWorld()->ModelByName(this->partnerName));
          if (this->partner)
            gzmsg << "Found partner: " << this->partnerName << std::endl;
        }

        if (!this->partner)
          return;

        ignition::math::Pose3d partnerPose = this->partner->WorldPose();
        double partnerYaw = partnerPose.Rot().Yaw();
        double partnerHeading = partnerYaw - 1.5707;
        ignition::math::Vector3d forward(std::cos(partnerHeading), std::sin(partnerHeading), 0);
        if (forward.Length() < 1e-3)
          forward = ignition::math::Vector3d(1, 0, 0);
        forward.Normalize();

        if (this->behavior == WALKING_PAIR)
        {
          if (!this->hasLastPartnerDir)
          {
            this->lastPartnerDir = forward;
            this->hasLastPartnerDir = true;
          }
          else
          {
            double dot = forward.X() * this->lastPartnerDir.X() + forward.Y() * this->lastPartnerDir.Y();
            if (dot < -0.5)
            {
              this->pairLateralOffset = -this->pairLateralOffset;
              this->lastPartnerDir = forward;
            }
            else
            {
              this->lastPartnerDir = forward;
            }
          }
        }

        ignition::math::Vector3d lateral(-forward.Y(), forward.X(), 0);
        double offsetSign = (this->pairLateralOffset >= 0) ? 1.0 : -1.0;
        ignition::math::Vector3d desiredPos = partnerPose.Pos() + lateral * (this->formationDistance * offsetSign);

        ignition::math::Pose3d pose(
          desiredPos.X(),
          desiredPos.Y(),
          1.05,
          1.5707,
          0,
          partnerYaw);

        this->actor->SetWorldPose(pose, false, false);
        this->actor->SetScriptTime(this->partner->ScriptTime());
        this->currentPos = pose.Pos();
        this->currentPos.Z(0);
        this->lastTime = info.simTime.Double();
      }

      void MaintainStaticPose(const common::UpdateInfo &info)
      {
        ignition::math::Pose3d pose(
          this->currentPos.X(),
          this->currentPos.Y(),
          1.05,
          1.5707,
          0,
          this->initialYaw);

        if (!this->staticPoseInitialized || (info.simTime.Double() - this->lastTime) > 0.01)
        {
          this->actor->SetWorldPose(pose, false, false);
          this->staticPoseInitialized = true;
          this->lastTime = info.simTime.Double();
        }
      }
    
    public: void Load(physics::ModelPtr model, sdf::ElementPtr sdf)
    {
      this->actor = boost::dynamic_pointer_cast<physics::Actor>(model);
      
      this->ConfigurePairDefaults();

      // Read behavior type
      std::string behaviorStr = sdf->Get<std::string>("behavior", "walking_solo").first;
      if (behaviorStr == "static")
        this->behavior = STATIC;
      else if (behaviorStr == "static_pair")
        this->behavior = STATIC_PAIR;
      else if (behaviorStr == "walking_solo")
        this->behavior = WALKING_SOLO;
      else if (behaviorStr == "walking_pair")
        this->behavior = WALKING_PAIR;
      else
        this->behavior = WALKING_SOLO;
      
      // Common parameters
      this->velocity = sdf->Get<double>("velocity", 0.8).first;
      
      // Initialize based on behavior
      switch (this->behavior)
      {
        case STATIC:
          this->InitStatic(sdf);
          break;
        case STATIC_PAIR:
          this->InitStaticPair(sdf);
          break;
        case WALKING_SOLO:
          this->InitWalkingSolo(sdf);
          break;
        case WALKING_PAIR:
          this->InitWalkingPair(sdf);
          break;
      }
      
      this->targetIdx = 0;
      this->lastTime = 0;
      this->currentPos = this->actor->WorldPose().Pos();
      this->currentPos.Z(0);
      this->initialYaw = this->actor->WorldPose().Rot().Yaw();
      this->staticPoseInitialized = false;
      this->lastPartnerDir = ignition::math::Vector3d(0, 0, 0);
      this->hasLastPartnerDir = false;
      
      // Setup custom trajectory
      auto skelAnims = this->actor->SkeletonAnimations();
      if (skelAnims.find("walking") != skelAnims.end())
      {
        physics::TrajectoryInfoPtr trajectoryInfo(new physics::TrajectoryInfo());
        trajectoryInfo->type = "walking";
        trajectoryInfo->duration = 1.0;
        this->actor->SetCustomTrajectory(trajectoryInfo);
      }
      
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        [this](const common::UpdateInfo &info) { this->OnUpdate(info); });
      
      gzmsg << "EnhancedActorPlugin loaded for " << this->actor->GetName() 
            << " with behavior: " << behaviorStr << std::endl;
    }
    
    private: void InitStatic(sdf::ElementPtr sdf)
    {
      std::uint32_t yawSeed = this->ResolveSeed(sdf, false, 0u);
      double yaw = this->SampleYawFromSeed(yawSeed);
      this->ApplyStaticYaw(yaw);
      ignition::math::Vector3d sampledPos = this->SampleStaticXY(sdf, this->actor->GetName() + "|static_xy");
      this->ApplyStaticXY(sampledPos);
      gzmsg << "Actor will stand still at initial position with yaw " << yaw << "\n";
    }

    private: void InitStaticPair(sdf::ElementPtr sdf)
    {
      if (sdf->HasElement("partner"))
      {
        this->partnerName = sdf->Get<std::string>("partner");
        this->formationDistance = sdf->Get<double>("formation_distance", 0.8).first;
        this->isPairLeader = (this->actor->GetName() <= this->partnerName);
        const std::string firstName = std::min(this->actor->GetName(), this->partnerName);
        const std::string secondName = std::max(this->actor->GetName(), this->partnerName);
        this->pairKey = firstName + "|" + secondName;
        this->pairLateralOffset = (this->isPairLeader ? -0.5 : 0.5) * this->formationDistance;

        std::uint32_t yawSeed = HashString(this->pairKey + "|static_pair_yaw");
        yawSeed = this->MixWithScenarioSeeds(sdf, yawSeed);
        double yaw = this->SampleYawFromSeed(yawSeed);
        this->ApplyStaticYaw(yaw);
        if (this->isPairLeader)
        {
          ignition::math::Vector3d sampledPos = this->SampleStaticXY(sdf, this->pairKey + "|static_pair_xy");
          this->ApplyStaticXY(sampledPos);
        }

        gzmsg << "  Static pair partner: " << this->partnerName << std::endl;
      }
      else
      {
        gzerr << "STATIC_PAIR behavior requires 'partner' parameter!\n";
        this->behavior = STATIC;
      }
    }
    
    private: void InitWalkingSolo(sdf::ElementPtr sdf, bool deterministic = false, std::uint32_t seed = 0)
    {
      double minX = sdf->Get<double>("min_x", -5.0).first;
      double maxX = sdf->Get<double>("max_x", 5.0).first;
      double minY = sdf->Get<double>("min_y", -5.0).first;
      double maxY = sdf->Get<double>("max_y", 5.0).first;
      int n = sdf->Get<int>("num_waypoints", 5).first;
      double minSep = sdf->Get<double>("min_waypoint_separation", -1.0).first;
      if (minSep <= 0.0)
      {
        double roomW = maxX - minX;
        double roomH = maxY - minY;
        minSep = 0.5 * std::sqrt(roomW * roomW + roomH * roomH);
      }
      minSep = std::min(minSep, std::sqrt((maxX - minX) * (maxX - minX) + (maxY - minY) * (maxY - minY)));
      std::uint32_t finalSeed = this->ResolveSeed(sdf, deterministic, seed);
      std::mt19937 rng(finalSeed);
      std::uniform_real_distribution<double> rx(minX, maxX);
      std::uniform_real_distribution<double> ry(minY, maxY);
      
      for (int i = 0; i < n; i++)
      {
        ignition::math::Vector3d candidate;
        bool accepted = false;
        ignition::math::Vector3d chosen;
        ignition::math::Vector3d bestCandidate;
        double bestDist = -1.0;
        for (int attempt = 0; attempt < 25; ++attempt)
        {
          candidate.Set(rx(rng), ry(rng), 0);
          if (i == 0)
          {
            chosen = candidate;
            accepted = true;
            break;
          }
          double dist = (candidate - waypoints.back()).Length();
          if (dist > bestDist)
          {
            bestDist = dist;
            bestCandidate = candidate;
          }
          if (dist >= minSep)
          {
            chosen = candidate;
            accepted = true;
            break;
          }
        }
        if (!accepted)
        {
          if (bestDist >= 0.0)
            chosen = bestCandidate;
          else
            chosen = ignition::math::Vector3d(rx(rng), ry(rng), 0);
        }
        waypoints.push_back(chosen);
        gzmsg << "  Waypoint " << i << ": " << waypoints[i] << std::endl;
      }
    }
    
    private: void InitWalkingPair(sdf::ElementPtr sdf)
    {
      // Read partner name
      if (sdf->HasElement("partner"))
      {
        this->partnerName = sdf->Get<std::string>("partner");
        this->formationDistance = sdf->Get<double>("formation_distance", 0.8).first;
        this->isPairLeader = (this->actor->GetName() <= this->partnerName);
        const std::string firstName = std::min(this->actor->GetName(), this->partnerName);
        const std::string secondName = std::max(this->actor->GetName(), this->partnerName);
        this->pairKey = firstName + "|" + secondName;
        std::uint32_t pairSeed = HashString(this->pairKey);
        this->pairLateralOffset = (this->isPairLeader ? -0.5 : 0.5) * this->formationDistance;

        gzmsg << "  Partner: " << this->partnerName << " (key: " << this->pairKey << ")" << std::endl;
        
        this->InitWalkingSolo(sdf, true, pairSeed);
      }
      else
      {
        gzerr << "WALKING_PAIR behavior requires 'partner' parameter!\n";
        this->behavior = WALKING_SOLO;
        this->InitWalkingSolo(sdf);
      }
    }
    
    private: void OnUpdate(const common::UpdateInfo &info)
    {
      if (this->behavior == STATIC)
      {
        this->MaintainStaticPose(info);
        return;
      }

      if (this->behavior == STATIC_PAIR)
      {
        if (this->isPairLeader)
          this->MaintainStaticPose(info);
        else
          this->UpdatePairFollower(info);
        return;
      }
      
      if (this->behavior == WALKING_PAIR && !this->isPairLeader)
      {
        this->UpdatePairFollower(info);
        return;
      }

      if (waypoints.empty()) return;
      
      double t = info.simTime.Double();
      double dt = t - this->lastTime;
      
      if (dt < 0.001) return;
      
      ignition::math::Vector3d target = waypoints[targetIdx];
      ignition::math::Vector3d forwardDir = target - this->currentPos;
      forwardDir.Z(0);
      if (forwardDir.Length() < 1e-3)
        forwardDir = ignition::math::Vector3d(1, 0, 0);
      
      if (this->behavior == WALKING_PAIR)
      {
        if (!this->partner)
        {
          this->partner = boost::dynamic_pointer_cast<physics::Actor>(
            this->actor->GetWorld()->ModelByName(this->partnerName));
          if (this->partner)
            gzmsg << "Found partner: " << this->partnerName << std::endl;
        }

        ignition::math::Vector3d segment(1, 0, 0);
        if (this->waypoints.size() >= 2)
        {
          auto prevIdx = (this->targetIdx + this->waypoints.size() - 1) % this->waypoints.size();
          segment = target - this->waypoints[prevIdx];
          segment.Z(0);
          if (segment.Length() < 1e-3)
            segment = ignition::math::Vector3d(1, 0, 0);
        }
        forwardDir = segment;
        ignition::math::Vector3d lateral(-segment.Y(), segment.X(), 0);
        if (lateral.Length() > 1e-3)
        {
          lateral.Normalize();
        }
        else
        {
          lateral = ignition::math::Vector3d(0, this->pairLateralOffset >= 0 ? 1.0 : -1.0, 0);
        }
        target += lateral * this->pairLateralOffset;
      }
      
      // For pair walking, check if we need to get partner's target
      ignition::math::Vector3d delta = target - currentPos;
      delta.Z(0);
      
      double dist = delta.Length();
      
      if (dist < 0.3)
      {
        targetIdx = (targetIdx + 1) % waypoints.size();
        
        gzmsg << this->actor->GetName() << " reached waypoint, next: " << targetIdx << std::endl;
        lastTime = t;
        return;
      }
      
      delta /= dist;  // normalize
      
      double step = velocity * dt;
      currentPos += delta * step;
      
      if (forwardDir.Length() > 1e-3)
      {
        forwardDir.Normalize();
      }
      double yaw = atan2(forwardDir.Y(), forwardDir.X());
      
      ignition::math::Pose3d pose(
        currentPos.X(), 
        currentPos.Y(), 
        1.05,
        1.5707,
        0,
        yaw + 1.5707
      );
      
      double distanceTraveled = (pose.Pos() - this->actor->WorldPose().Pos()).Length();
      
      this->actor->SetWorldPose(pose, false, false);
      this->actor->SetScriptTime(this->actor->ScriptTime() + (distanceTraveled * 4.5));
      
      lastTime = t;
    }
  };
  
  GZ_REGISTER_MODEL_PLUGIN(EnhancedActorPlugin)
}