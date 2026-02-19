package info.openrocket.exporter;

import com.google.inject.Guice;
import com.google.inject.Injector;
import info.openrocket.core.aerodynamics.AerodynamicForces;
import info.openrocket.core.aerodynamics.BarrowmanCalculator;
import info.openrocket.core.aerodynamics.FlightConditions;
import info.openrocket.core.document.OpenRocketDocument;
import info.openrocket.core.document.Simulation;
import info.openrocket.core.file.GeneralRocketLoader;
import info.openrocket.core.logging.WarningSet;
import info.openrocket.core.masscalc.MassCalculator;
import info.openrocket.core.masscalc.RigidBody;
import info.openrocket.core.models.atmosphere.AtmosphericConditions;
import info.openrocket.core.models.atmosphere.ExtendedISAModel;
import info.openrocket.core.motor.Motor;
import info.openrocket.core.motor.MotorConfiguration;
import info.openrocket.core.motor.ThrustCurveMotor;
import info.openrocket.core.plugin.PluginModule;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.simulation.FlightData;
import info.openrocket.core.simulation.FlightDataBranch;
import info.openrocket.core.simulation.FlightDataType;
import info.openrocket.core.simulation.SimulationOptions;
import info.openrocket.core.startup.Application;
import info.openrocket.core.startup.CoreModule;
import info.openrocket.core.util.CoordinateIF;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import picocli.CommandLine;
import picocli.CommandLine.Command;
import picocli.CommandLine.Option;
import us.hebi.matlab.mat.format.Mat5;
import us.hebi.matlab.mat.types.MatFile;
import us.hebi.matlab.mat.types.Matrix;
import us.hebi.matlab.mat.types.Sinks;

import java.io.File;
import java.util.*;
import java.util.concurrent.Callable;

/**
 * Command-line tool to export OpenRocket .ork files to MATLAB .mat files.
 * <p>
 * Extracts mass properties (time-varying), thrust curve, aerodynamic coefficient
 * tables (CD, CN, Cm, Croll vs Mach × AoA), reference geometry, simulation settings,
 * and optionally runs the OpenRocket simulation for validation data.
 * <p>
 * Usage:
 * <pre>
 *   java -jar ork-exporter.jar --ork rocket.ork --out rocket.mat
 * </pre>
 */
@Command(name = "ork-exporter",
        mixinStandardHelpOptions = true,
        version = "1.0",
        description = "Export OpenRocket .ork files to MATLAB .mat format for Simulink integration.")
public class OrkExporter implements Callable<Integer> {

    private static final Logger logger = LoggerFactory.getLogger(OrkExporter.class);

    @Option(names = {"--ork"}, required = true, description = "Path to the .ork file")
    private File orkFile;

    @Option(names = {"--out"}, description = "Output .mat file path (default: <ork_name>.mat)")
    private File outFile;

    @Option(names = {"--sim"}, defaultValue = "0", description = "Simulation index to use (0-based, default: 0)")
    private int simIndex;

    @Option(names = {"--mach-max"}, defaultValue = "3.0", description = "Maximum Mach number for CD table (default: 3.0)")
    private double machMax;

    @Option(names = {"--mach-step"}, defaultValue = "0.01", description = "Mach step size for CD table (default: 0.01)")
    private double machStep;

    @Option(names = {"--aoa-max"}, defaultValue = "15.0", description = "Maximum angle of attack in degrees (default: 15.0)")
    private double aoaMaxDeg;

    @Option(names = {"--aoa-step"}, defaultValue = "1.0", description = "AoA step size in degrees (default: 1.0)")
    private double aoaStepDeg;

    @Option(names = {"--mass-dt"}, defaultValue = "0.01", description = "Time step for mass-vs-time sampling in seconds (default: 0.01)")
    private double massDt;

    @Option(names = {"--run-sim"}, defaultValue = "false", description = "Run OpenRocket simulation and include validation data")
    private boolean runSim;

    @Option(names = {"--altitude"}, defaultValue = "0.0", description = "Reference altitude for atmospheric conditions in CD computation (default: 0 m)")
    private double refAltitude;

    public static void main(String[] args) {
        int exitCode = new CommandLine(new OrkExporter()).execute(args);
        System.exit(exitCode);
    }

    @Override
    public Integer call() throws Exception {
        // Validate inputs
        if (!orkFile.exists()) {
            logger.error("ORK file not found: {}", orkFile.getAbsolutePath());
            return 1;
        }
        if (outFile == null) {
            String name = orkFile.getName();
            int dot = name.lastIndexOf('.');
            String base = (dot > 0) ? name.substring(0, dot) : name;
            outFile = new File(orkFile.getParentFile(), base + ".mat");
        }

        // Bootstrap OpenRocket core headlessly
        logger.info("Initializing OpenRocket core...");
        initOpenRocket();

        // Load the .ork file
        logger.info("Loading .ork file: {}", orkFile.getAbsolutePath());
        GeneralRocketLoader loader = new GeneralRocketLoader(orkFile);
        OpenRocketDocument doc = loader.load();
        WarningSet warnings = loader.getWarnings();
        if (warnings != null && warnings.size() > 0) {
            logger.warn("Loader warnings: {}", warnings);
        }

        Rocket rocket = doc.getRocket();
        logger.info("Rocket: {}", rocket.getName());

        // Select simulation and flight configuration
        List<Simulation> sims = doc.getSimulations();
        if (sims.isEmpty()) {
            logger.error("No simulations found in the .ork file. Cannot determine flight configuration.");
            return 1;
        }
        if (simIndex >= sims.size()) {
            logger.error("Simulation index {} out of range. File has {} simulation(s).", simIndex, sims.size());
            return 1;
        }
        Simulation sim = sims.get(simIndex);
        FlightConfigurationId fcid = sim.getFlightConfigurationId();
        FlightConfiguration config = rocket.getFlightConfiguration(fcid);
        config.setAllStages();
        SimulationOptions simOptions = sim.getOptions();

        logger.info("Using simulation: '{}' (index {})", sim.getName(), simIndex);
        logger.info("Flight configuration: {}", config.getName());

        // Gather all data into a MatFile
        MatFile matFile = Mat5.newMatFile();

        // 1. Extract motor data and thrust curve
        extractMotorData(config, matFile);

        // 2. Extract mass & inertia (time-varying)
        extractMassData(config, matFile);

        // 3. Extract aerodynamic coefficient tables (2D: Mach × AoA)
        extractAeroData(config, matFile);

        // 4. Extract reference geometry
        extractGeometry(config, rocket, matFile);

        // 5. Extract simulation settings
        extractSimSettings(simOptions, matFile);

        // 6. Optionally run OR simulation for validation
        if (runSim) {
            extractSimResults(sim, matFile);
        }

        // Write the .mat file
        logger.info("Writing MAT file: {}", outFile.getAbsolutePath());
        matFile.writeTo(Sinks.newStreamingFile(outFile));
        matFile.close();
        logger.info("Export complete.");

        return 0;
    }

    /**
     * Initialize OpenRocket's Guice dependency injection headlessly.
     */
    private void initOpenRocket() {
        CoreModule coreModule = new CoreModule();
        Injector injector = Guice.createInjector(coreModule, new PluginModule());
        Application.setInjector(injector);
        coreModule.startLoader();

        // Wait for motor database to finish loading
        try {
            // Give the background loader a moment to initialize
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /**
     * Extract motor thrust curve and metadata.
     */
    private void extractMotorData(FlightConfiguration config, MatFile matFile) {
        Collection<MotorConfiguration> motorConfigs = config.getActiveMotors();
        int nMotors = motorConfigs.size();

        if (nMotors == 0) {
            logger.warn("No active motors in the flight configuration.");
            matFile.addArray("thrust_time", Mat5.newScalar(0));
            matFile.addArray("thrust_force", Mat5.newScalar(0));
            matFile.addArray("n_motors", Mat5.newScalar(0));
            matFile.addArray("burn_time", Mat5.newScalar(0));
            matFile.addArray("motor_name", Mat5.newString("none"));
            return;
        }

        // Use the first motor for the thrust curve shape; multiply by nMotors for clusters
        MotorConfiguration firstMC = motorConfigs.iterator().next();
        Motor motor = firstMC.getMotor();
        String motorName = motor.getDesignation();
        double burnTime = motor.getBurnTime();

        logger.info("Motor: {} (x{}), burn time: {} s", motorName, nMotors, burnTime);

        // Get raw thrust curve
        double[] timePoints;
        double[] thrustPoints;

        if (motor instanceof ThrustCurveMotor tcm) {
            timePoints = tcm.getTimePoints();
            thrustPoints = tcm.getThrustPoints();
        } else {
            // Fallback: sample the motor at regular intervals
            int nSamples = (int) Math.ceil(burnTime / 0.001) + 1;
            timePoints = new double[nSamples];
            thrustPoints = new double[nSamples];
            for (int i = 0; i < nSamples; i++) {
                timePoints[i] = i * 0.001;
                thrustPoints[i] = motor.getThrust(timePoints[i]);
            }
        }

        // Scale thrust by number of motors
        double[] totalThrust = new double[thrustPoints.length];
        for (int i = 0; i < thrustPoints.length; i++) {
            totalThrust[i] = thrustPoints[i] * nMotors;
        }

        // Ensure the curve ends at zero thrust (append if needed)
        int len = timePoints.length;
        if (totalThrust[len - 1] > 0.001) {
            timePoints = Arrays.copyOf(timePoints, len + 1);
            totalThrust = Arrays.copyOf(totalThrust, len + 1);
            timePoints[len] = timePoints[len - 1] + 0.001;
            totalThrust[len] = 0.0;
        }

        // Write to matFile
        matFile.addArray("thrust_time", toRowVector(timePoints));
        matFile.addArray("thrust_force", toRowVector(totalThrust));
        matFile.addArray("n_motors", Mat5.newScalar(nMotors));
        matFile.addArray("burn_time", Mat5.newScalar(burnTime));
        matFile.addArray("motor_name", Mat5.newString(motorName));
        matFile.addArray("motor_launch_mass", Mat5.newScalar(motor.getLaunchMass()));
        matFile.addArray("motor_burnout_mass", Mat5.newScalar(motor.getBurnoutMass()));
        matFile.addArray("motor_diameter", Mat5.newScalar(motor.getDiameter()));
        matFile.addArray("motor_length", Mat5.newScalar(motor.getLength()));
    }

    /**
     * Extract time-varying mass, inertia, and CG data.
     */
    private void extractMassData(FlightConfiguration config, MatFile matFile) {
        // Get structural (dry) properties
        RigidBody structure = MassCalculator.calculateStructure(config);
        RigidBody launchBody = MassCalculator.calculateLaunch(config);
        RigidBody burnoutBody = MassCalculator.calculateBurnout(config);

        double structureMass = structure.getMass();
        double launchMass = launchBody.getMass();
        double burnoutMass = burnoutBody.getMass();

        logger.info("Structure mass: {} kg", structureMass);
        logger.info("Launch mass:    {} kg", launchMass);
        logger.info("Burnout mass:   {} kg", burnoutMass);

        // Static mass/inertia values
        matFile.addArray("mass_launch", Mat5.newScalar(launchMass));
        matFile.addArray("mass_burnout", Mat5.newScalar(burnoutMass));
        matFile.addArray("mass_structure", Mat5.newScalar(structureMass));

        matFile.addArray("Ixx_launch", Mat5.newScalar(launchBody.getIxx()));
        matFile.addArray("Iyy_launch", Mat5.newScalar(launchBody.getIyy()));
        matFile.addArray("Izz_launch", Mat5.newScalar(launchBody.getIzz()));

        matFile.addArray("cg_launch_x", Mat5.newScalar(launchBody.getCM().getX()));

        // Time-varying mass from motor burn
        Collection<MotorConfiguration> motorConfigs = config.getActiveMotors();
        if (motorConfigs.isEmpty()) {
            // No motors — constant mass
            matFile.addArray("mass_time", toRowVector(new double[]{0}));
            matFile.addArray("mass_total", toRowVector(new double[]{structureMass}));
            matFile.addArray("mass_propellant", toRowVector(new double[]{0}));
            matFile.addArray("Ixx_vs_time", toRowVector(new double[]{structure.getIxx()}));
            matFile.addArray("Iyy_vs_time", toRowVector(new double[]{structure.getIyy()}));
            matFile.addArray("Izz_vs_time", toRowVector(new double[]{structure.getIzz()}));
            matFile.addArray("cg_x_vs_time", toRowVector(new double[]{structure.getCM().getX()}));
            return;
        }

        // Find the longest burning motor
        double maxBurnTime = 0;
        for (MotorConfiguration mc : motorConfigs) {
            maxBurnTime = Math.max(maxBurnTime, mc.getMotor().getBurnTime());
        }

        // Sample time-varying mass and inertia
        int nSteps = (int) Math.ceil(maxBurnTime / massDt) + 2; // +1 for t=0, +1 for post-burnout
        double[] massTime = new double[nSteps];
        double[] massTotal = new double[nSteps];
        double[] massPropellant = new double[nSteps];
        double[] ixxTime = new double[nSteps];
        double[] iyyTime = new double[nSteps];
        double[] izzTime = new double[nSteps];
        double[] cgxTime = new double[nSteps];

        for (int i = 0; i < nSteps; i++) {
            double t = Math.min(i * massDt, maxBurnTime);
            if (i == nSteps - 1) {
                t = maxBurnTime; // Ensure last point is exactly at burnout
            }
            massTime[i] = t;

            // Compute motor mass and inertia at time t
            double motorMassTotal = 0;
            double motorIxx = 0;
            double motorIyy = 0;
            double motorIzz = 0;
            double motorMassCgXWeighted = 0;
            double propMass = 0;

            for (MotorConfiguration mc : motorConfigs) {
                Motor m = mc.getMotor();
                double mMass = m.getTotalMass(t);
                double mProp = m.getPropellantMass(t);
                double mCGx = m.getCMx(t);

                motorMassTotal += mMass;
                propMass += mProp;
                motorMassCgXWeighted += mMass * mCGx;

                // Motor inertia = unitInertia * mass
                motorIxx += m.getUnitIxx() * mMass;
                motorIyy += m.getUnitIyy() * mMass;
                motorIzz += m.getUnitIzz() * mMass;
            }

            massTotal[i] = structureMass + motorMassTotal;
            massPropellant[i] = propMass;

            // Combined CG (1D along x-axis): weighted average
            double totalMass = massTotal[i];
            if (totalMass > 0) {
                cgxTime[i] = (structureMass * structure.getCM().getX() + motorMassCgXWeighted) / totalMass;
            } else {
                cgxTime[i] = structure.getCM().getX();
            }

            // Combined inertia using parallel axis theorem
            // Structure inertia (already about structural CG)
            double dx_struct = structure.getCM().getX() - cgxTime[i];
            ixxTime[i] = structure.getIxx() + motorIxx; // Axial: no parallel axis offset
            iyyTime[i] = structure.getIyy() + structureMass * dx_struct * dx_struct
                    + motorIyy; // Simplified: motor PAT would require per-motor CG offset
            izzTime[i] = structure.getIzz() + structureMass * dx_struct * dx_struct
                    + motorIzz;
        }

        matFile.addArray("mass_time", toRowVector(massTime));
        matFile.addArray("mass_total", toRowVector(massTotal));
        matFile.addArray("mass_propellant", toRowVector(massPropellant));
        matFile.addArray("Ixx_vs_time", toRowVector(ixxTime));
        matFile.addArray("Iyy_vs_time", toRowVector(iyyTime));
        matFile.addArray("Izz_vs_time", toRowVector(izzTime));
        matFile.addArray("cg_x_vs_time", toRowVector(cgxTime));
    }

    /**
     * Extract 2D aerodynamic coefficient tables: CD, CN, Cm, Croll vs Mach × AoA.
     */
    private void extractAeroData(FlightConfiguration config, MatFile matFile) {
        BarrowmanCalculator calc = new BarrowmanCalculator();
        ExtendedISAModel atm = new ExtendedISAModel();
        AtmosphericConditions atmCond = atm.getConditions(refAltitude);

        // Build breakpoint arrays
        int nMach = (int) Math.round(machMax / machStep) + 1;
        int nAoA = (int) Math.round(aoaMaxDeg / aoaStepDeg) + 1;

        double[] machBp = new double[nMach];
        double[] aoaBp = new double[nAoA]; // in radians

        for (int i = 0; i < nMach; i++) {
            machBp[i] = i * machStep;
        }
        for (int j = 0; j < nAoA; j++) {
            aoaBp[j] = Math.toRadians(j * aoaStepDeg);
        }

        // Avoid Mach = 0 exactly (can cause division issues in some calcs)
        if (machBp[0] < 1e-6) {
            machBp[0] = 0.001;
        }

        logger.info("Computing aero tables: {} Mach points x {} AoA points = {} evaluations",
                nMach, nAoA, nMach * nAoA);

        // Allocate 2D tables (stored row-major: machIdx × aoaIdx)
        double[][] cdTable = new double[nMach][nAoA];
        double[][] cnTable = new double[nMach][nAoA];
        double[][] cmTable = new double[nMach][nAoA];
        double[][] crollTable = new double[nMach][nAoA];
        double[][] cdAxialTable = new double[nMach][nAoA];
        double[][] pressureCdTable = new double[nMach][nAoA];
        double[][] frictionCdTable = new double[nMach][nAoA];
        double[][] baseCdTable = new double[nMach][nAoA];

        WarningSet aeroWarnings = new WarningSet();

        for (int i = 0; i < nMach; i++) {
            for (int j = 0; j < nAoA; j++) {
                FlightConditions fc = new FlightConditions(config);
                fc.setMach(machBp[i]);
                fc.setAOA(aoaBp[j]);
                fc.setAtmosphericConditions(atmCond);
                fc.setRollRate(0);
                fc.setPitchRate(0);
                fc.setYawRate(0);

                aeroWarnings.clear();
                try {
                    AerodynamicForces forces = calc.getAerodynamicForces(config, fc, aeroWarnings);
                    cdTable[i][j] = forces.getCD();
                    cnTable[i][j] = forces.getCN();
                    cmTable[i][j] = forces.getCm();
                    crollTable[i][j] = forces.getCroll();
                    cdAxialTable[i][j] = forces.getCDaxial();
                    pressureCdTable[i][j] = forces.getPressureCD();
                    frictionCdTable[i][j] = forces.getFrictionCD();
                    baseCdTable[i][j] = forces.getBaseCD();
                } catch (Exception e) {
                    logger.warn("Aero calc failed at Mach={}, AoA={}°: {}", machBp[i],
                            Math.toDegrees(aoaBp[j]), e.getMessage());
                    cdTable[i][j] = Double.NaN;
                    cnTable[i][j] = Double.NaN;
                    cmTable[i][j] = Double.NaN;
                    crollTable[i][j] = Double.NaN;
                    cdAxialTable[i][j] = Double.NaN;
                }
            }
            // Progress
            if (i > 0 && i % 50 == 0) {
                logger.info("  Aero progress: {}/{} Mach points", i, nMach);
            }
        }

        matFile.addArray("mach_bp", toRowVector(machBp));
        matFile.addArray("aoa_bp", toRowVector(aoaBp));
        matFile.addArray("CD_table", toMatrix(cdTable));
        matFile.addArray("CN_table", toMatrix(cnTable));
        matFile.addArray("Cm_table", toMatrix(cmTable));
        matFile.addArray("Croll_table", toMatrix(crollTable));
        matFile.addArray("CDaxial_table", toMatrix(cdAxialTable));
        matFile.addArray("pressureCD_table", toMatrix(pressureCdTable));
        matFile.addArray("frictionCD_table", toMatrix(frictionCdTable));
        matFile.addArray("baseCD_table", toMatrix(baseCdTable));
    }

    /**
     * Extract reference geometry from the flight configuration and rocket.
     */
    private void extractGeometry(FlightConfiguration config, Rocket rocket, MatFile matFile) {
        double refLength = config.getReferenceLength();
        double refArea = config.getReferenceArea();

        matFile.addArray("ref_length", Mat5.newScalar(refLength));
        matFile.addArray("ref_area", Mat5.newScalar(refArea));

        // Compute total rocket length by iterating the component tree
        double totalLength = 0;
        int finCount = 0;
        double finRootChord = 0, finSpan = 0, finSweep = 0;

        for (RocketComponent comp : rocket) {
            if (comp instanceof AxialStage || comp instanceof BodyTube ||
                    comp instanceof NoseCone || comp instanceof Transition) {
                totalLength += comp.getLength();
            }
            if (comp instanceof TrapezoidFinSet tfs) {
                finCount = tfs.getFinCount();
                finRootChord = tfs.getRootChord();
                finSpan = tfs.getHeight();
                finSweep = tfs.getSweep();
            } else if (comp instanceof EllipticalFinSet efs) {
                finCount = efs.getFinCount();
                finRootChord = efs.getLength();
                finSpan = efs.getHeight();
            } else if (comp instanceof FinSet fs) {
                finCount = fs.getFinCount();
                finSpan = fs.getLength();
            }
        }

        matFile.addArray("rocket_length", Mat5.newScalar(totalLength));
        matFile.addArray("fin_count", Mat5.newScalar(finCount));
        matFile.addArray("fin_root_chord", Mat5.newScalar(finRootChord));
        matFile.addArray("fin_span", Mat5.newScalar(finSpan));
        matFile.addArray("fin_sweep", Mat5.newScalar(finSweep));
    }

    /**
     * Extract simulation settings (launch rod, atmosphere, wind).
     */
    private void extractSimSettings(SimulationOptions opts, MatFile matFile) {
        matFile.addArray("launch_rod_length", Mat5.newScalar(opts.getLaunchRodLength()));
        matFile.addArray("launch_rod_angle", Mat5.newScalar(opts.getLaunchRodAngle()));
        matFile.addArray("launch_rod_direction", Mat5.newScalar(opts.getLaunchRodDirection()));
        matFile.addArray("launch_altitude", Mat5.newScalar(opts.getLaunchAltitude()));
        matFile.addArray("launch_latitude", Mat5.newScalar(opts.getLaunchLatitude()));
        matFile.addArray("launch_longitude", Mat5.newScalar(opts.getLaunchLongitude()));
        matFile.addArray("launch_temperature", Mat5.newScalar(opts.getLaunchTemperature()));
        matFile.addArray("launch_pressure", Mat5.newScalar(opts.getLaunchPressure()));
        matFile.addArray("time_step", Mat5.newScalar(opts.getTimeStep()));
    }

    /**
     * Run the OpenRocket simulation and extract results for validation.
     */
    private void extractSimResults(Simulation sim, MatFile matFile) {
        logger.info("Running OpenRocket simulation for validation...");
        try {
            sim.simulate();
            FlightData data = sim.getSimulatedData();
            if (data == null) {
                logger.warn("Simulation produced no data.");
                return;
            }

            // Summary values
            matFile.addArray("or_max_altitude", Mat5.newScalar(safeGet(data::getMaxAltitude)));
            matFile.addArray("or_max_velocity", Mat5.newScalar(safeGet(data::getMaxVelocity)));
            matFile.addArray("or_max_acceleration", Mat5.newScalar(safeGet(data::getMaxAcceleration)));
            matFile.addArray("or_max_mach", Mat5.newScalar(safeGet(data::getMaxMachNumber)));
            matFile.addArray("or_time_to_apogee", Mat5.newScalar(safeGet(data::getTimeToApogee)));
            matFile.addArray("or_flight_time", Mat5.newScalar(safeGet(data::getFlightTime)));

            // Time-series from the first branch (sustainer)
            if (data.getBranchCount() > 0) {
                FlightDataBranch branch = data.getBranch(0);
                List<Double> orTime = branch.get(FlightDataType.TYPE_TIME);
                List<Double> orAlt = branch.get(FlightDataType.TYPE_ALTITUDE);
                List<Double> orVel = branch.get(FlightDataType.TYPE_VELOCITY_TOTAL);
                List<Double> orAccel = branch.get(FlightDataType.TYPE_ACCELERATION_TOTAL);
                List<Double> orMach = branch.get(FlightDataType.TYPE_MACH_NUMBER);
                List<Double> orMass = branch.get(FlightDataType.TYPE_MASS);
                List<Double> orThrust = branch.get(FlightDataType.TYPE_THRUST_FORCE);
                List<Double> orDrag = branch.get(FlightDataType.TYPE_DRAG_FORCE);
                List<Double> orCd = branch.get(FlightDataType.TYPE_DRAG_COEFF);

                if (orTime != null && !orTime.isEmpty()) {
                    matFile.addArray("or_time", toRowVectorFromList(orTime));
                }
                if (orAlt != null && !orAlt.isEmpty()) {
                    matFile.addArray("or_altitude", toRowVectorFromList(orAlt));
                }
                if (orVel != null && !orVel.isEmpty()) {
                    matFile.addArray("or_velocity", toRowVectorFromList(orVel));
                }
                if (orAccel != null && !orAccel.isEmpty()) {
                    matFile.addArray("or_acceleration", toRowVectorFromList(orAccel));
                }
                if (orMach != null && !orMach.isEmpty()) {
                    matFile.addArray("or_mach", toRowVectorFromList(orMach));
                }
                if (orMass != null && !orMass.isEmpty()) {
                    matFile.addArray("or_mass", toRowVectorFromList(orMass));
                }
                if (orThrust != null && !orThrust.isEmpty()) {
                    matFile.addArray("or_thrust", toRowVectorFromList(orThrust));
                }
                if (orDrag != null && !orDrag.isEmpty()) {
                    matFile.addArray("or_drag", toRowVectorFromList(orDrag));
                }
                if (orCd != null && !orCd.isEmpty()) {
                    matFile.addArray("or_cd", toRowVectorFromList(orCd));
                }
            }
            logger.info("OpenRocket simulation complete. Max altitude: {} m", safeGet(data::getMaxAltitude));
        } catch (Exception e) {
            logger.error("OpenRocket simulation failed: {}", e.getMessage(), e);
        }
    }

    // ─── Utility methods ─────────────────────────────────────────────

    /**
     * Convert a double[] to a 1×N MATLAB row vector.
     */
    private Matrix toRowVector(double[] data) {
        Matrix m = Mat5.newMatrix(1, data.length);
        for (int i = 0; i < data.length; i++) {
            m.setDouble(0, i, data[i]);
        }
        return m;
    }

    /**
     * Convert a List<Double> to a 1×N MATLAB row vector.
     */
    private Matrix toRowVectorFromList(List<Double> data) {
        Matrix m = Mat5.newMatrix(1, data.size());
        for (int i = 0; i < data.size(); i++) {
            m.setDouble(0, i, data.get(i));
        }
        return m;
    }

    /**
     * Convert a double[][] (nRows × nCols) to a MATLAB matrix.
     * MATLAB stores column-major, so we need to set elements accordingly.
     */
    private Matrix toMatrix(double[][] data) {
        int nRows = data.length;
        int nCols = data[0].length;
        Matrix m = Mat5.newMatrix(nRows, nCols);
        for (int i = 0; i < nRows; i++) {
            for (int j = 0; j < nCols; j++) {
                m.setDouble(i, j, data[i][j]);
            }
        }
        return m;
    }

    /**
     * Safely get a value from a supplier, returning NaN on failure.
     */
    private double safeGet(java.util.function.DoubleSupplier supplier) {
        try {
            return supplier.getAsDouble();
        } catch (Exception e) {
            return Double.NaN;
        }
    }
}
