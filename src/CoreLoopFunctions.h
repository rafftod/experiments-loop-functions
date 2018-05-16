/*
 * @file <src/core/CoreLoopFunctions.h>
 *
 * @author Antoine Ligot - <aligot@ulb.ac.be>
 *
 * @package ARGoS3-AutoMoDe
 *
 * @license MIT License
 */

#ifndef AUTOMODE_LOOP_FUNCTIONS
#define AUTOMODE_LOOP_FUNCTIONS

#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>

using namespace argos;

class CoreLoopFunctions: public CLoopFunctions {
  protected:

    ~CoreLoopFunctions();

    /*
     * Initialization method where the parameters of the loop function
     * are fetched from the xml declaration.
     */
    virtual void Init(argos::TConfigurationNode& t_tree);

    /*
     * Reset function. First remove the robots, then re-position them.
     */
    virtual void Reset();

    /*
     * Method used to remove the robots from the arena.
     */
    void RemoveRobots();

    /*
     * Method used to reallocate the robots.
     * The position is given by the method GetRandomPosition().
     */
    void MoveRobots();

    /*
     * Method used to create and distribute the robots.
     * The position is given by the method GetRandomPosition().
     */
    void PositionRobots();

    /*
     * The number of robots to be placed for the experiment.
     */
    UInt32 m_unNumberRobots;

    /*
     * The radius of the circle in which the robots will be distributed.
     */
    Real m_fDistributionRadius;

    CRandom::CRNG* m_pcRng;

  public:

    /*
     * This method should return the fitness of the controller.
     */
    virtual Real GetObjectiveFunction() = 0;

    /*
     * Return a random position.
     */
    virtual CVector3 GetRandomPosition() = 0;

    /*
     * Id of controller to be specified in the parameters of the definition
     * of the loop function in the .argos file.
     */
    std::string m_strControllerId;

};

#endif
