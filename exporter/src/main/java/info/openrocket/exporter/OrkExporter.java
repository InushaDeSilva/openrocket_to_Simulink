package info.openrocket.exporter;

import com.google.inject.Guice;
import com.google.inject.Injector;
import info.openrocket.core.aerodynamics.*;
import info.openrocket.core.document.OpenRocketDocument;
import info.openrocket.core.document.Simulation;
import info.openrocket.core.file.GeneralRocketLoader;
import info.openrocket.core.logging.WarningSet;
import info.openrocket.core.masscalc.MassCalculator;
import info.openrocket.core.masscalc.RigidBody;
import info.openrocket.core.models.atmosphere.AtmosphericConditions;
import info.openrocket.core.models.atmosphere.ExtendedISAModel;
import info.openrocket.core.models.wind.PinkNoiseWindModel;
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
import info.openrocket.core.util.BuildProperties;
import info.openrocket.core.util.CoordinateIF;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import picocli.CommandLine;
import picocli.CommandLine.Command;
import picocli.CommandLine.Option;
import us.hebi.matlab.mat.format.Mat5;
import us.hebi.matlab.mat.types.*;
import us.hebi.matlab.mat.types.Sinks;

import java.io.File;
import java.io.FileInputStream;
import java.nio.file.Path;
import java.security.MessageDigest;
import java.time.Instant;
import java.util.*;
import java.util.concurrent.Callable;

/**
 * Command-line tool to export OpenRocket .ork files to MATLAB .mat files.
 * <p>
 * Extracts mass properties (time-varying with separated components), thrust curve,
 * aerodynamic coefficient tables (CD, CN, Cm, Croll, CrollDamp, CrollForce vs Mach x AoA),
 * reference geometry, per-finset geometry, simulation settings, metadata, dI/dt arrays,
 * and optionally runs the OpenRocket simulation for validation data.
 * <p>
 * v2.1 improvements:
 * <ul>
 *   <li>3D aero tables (Mach × AoA × Altitude) capturing Reynolds/density effects</li>
 *   <li>Motor identity (manufacturer, total impulse, avg/max thrust, type)</li>
 *   <li>Wind/atmosphere scenario data (speed, direction, turbulence, ISA, humidity)</li>
 *   <li>rocket_* / scenario_* partitioning of invariant vs per-flight data</li>
 *   <li>3×3 inertia tensor (I_tensor_vs_time) for Aerospace Blockset</li>
 *   <li>Computed total impulse via trapezoidal integration</li>
 *   <li>Rail exit velocity from OR simulation</li>
 *   <li>Simulation listing on invalid --sim index</li>
 *   <li>Coordinate frame mapping metadata for Aerospace Blockset (NED, x-fwd)</li>
 * </ul>
 * <p>
 * v2.0 improvements:
 * <ul>
 *   <li>Configurable aero model (--aero-model auto/barrowman)</li>
 *   <li>Metadata struct with coordinate conventions, hashing, versioning</li>
 *   <li>Monotonic time guarantee on thrust and mass time arrays</li>
 *   <li>Motor parallel-axis-theorem fix on Iyy/Izz</li>
 *   <li>Separated mass/inertia breakdown (structure, motor, total)</li>
 *   <li>dI/dt arrays for Custom Variable Mass 6DOF block</li>
 *   <li>Per-finset geometry export (fin0_*, fin1_*, ...)</li>
 *   <li>CrollDamp and CrollForce aero tables</li>
 *   <li>Additional sim result channels (AoA, roll rate, CG, CP, stability)</li>
 *   <li>Motor propellant mass and burnout tail with far-future hold</li>
 * </ul>
 * <p>
 * Usage:
 * <pre>
 *   java -jar ork-exporter.jar --ork rocket.ork --out rocket.mat
 * </pre>
 */
@Command(name = "ork-exporter",
        mixinStandardHelpOptions = true,
        version = "2.1.0",
        description = "Export OpenRocket .ork files to MATLAB .mat format for Simulink integration.")
public class OrkExporter implements Callable<Integer> {

    private static final Logger logger = LoggerFactory.getLogger(OrkExporter.class);

    /** Exporter version string embedded in metadata */
    public static final String EXPORTER_VERSION = "2.1.0";

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

    @Option(names = {"--aero-model"}, defaultValue = "auto",
            description = "Aerodynamic model: auto (match sim config), barrowman (force analytical). Default: auto")
    private String aeroModel;

    @Option(names = {"--alt-max"}, defaultValue = "5000.0",
            description = "Maximum altitude for 3D aero tables in meters (default: 5000)")
    private double altMax;

    @Option(names = {"--alt-step"}, defaultValue = "500.0",
            description = "Altitude step for 3D aero tables in meters (default: 500)")
    private double altStep;

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
            logger.error("Simulation index {} out of range. File has {} simulation(s):", simIndex, sims.size());
            for (int i = 0; i < sims.size(); i++) {
                Simulation s = sims.get(i);
                FlightConfigurationId fid = s.getFlightConfigurationId();
                FlightConfiguration fc = rocket.getFlightConfiguration(fid);
                Collection<MotorConfiguration> mcs = fc.getActiveMotors();
                String motorDesc = mcs.isEmpty() ? "(no motors)" :
                        mcs.iterator().next().getMotor().getDesignation();
                logger.error("  [{}] '{}' — config='{}' motor='{}'", i, s.getName(), fc.getName(), motorDesc);
            }
            return 1;
        }
        Simulation sim = sims.get(simIndex);
        FlightConfigurationId fcid = sim.getFlightConfigurationId();
        FlightConfiguration config = rocket.getFlightConfiguration(fcid);
        config.setAllStages();
        SimulationOptions simOptions = sim.getOptions();

        logger.info("Using simulation: '{}' (index {})", sim.getName(), simIndex);
        logger.info("Flight configuration: {}", config.getName());

        // Build the aerodynamic calculator (step 2)
        AerodynamicCalculator aeroCalc = buildAeroCalculator(simOptions);

        // Gather all data into a MatFile
        MatFile matFile = Mat5.newMatFile();

        // 0. Extract metadata
        extractMetadata(config, sim, simOptions, aeroCalc, matFile);

        // 1. Extract motor data and thrust curve
        extractMotorData(config, matFile);

        // 2. Extract mass & inertia (time-varying, separated, with dI/dt)
        extractMassData(config, matFile);

        // 3. Extract aerodynamic coefficient tables (3D: Mach x AoA x Altitude)
        extractAeroData(config, aeroCalc, matFile);

        // 4. Extract reference geometry (per-finset)
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

    // --- Step 2: Build Aero Calculator ---

    /**
     * Build an AerodynamicCalculator matching the simulation configuration.
     * <p>
     * When aeroModel is "auto", uses lookup tables if the simulation has them configured,
     * otherwise falls back to Barrowman analytical. When "barrowman", forces analytical.
     */
    private AerodynamicCalculator buildAeroCalculator(SimulationOptions simOptions) {
        if ("barrowman".equalsIgnoreCase(aeroModel)) {
            logger.info("Aero model: forced Barrowman (analytical)");
            return new BarrowmanCalculator();
        }

        // auto mode: check if simulation has lookup tables configured
        StabilityCalculator stabilityCalc;
        DragCalculator dragCalc;

        if (simOptions.hasDragLookup()) {
            logger.info("Aero model: auto -> using drag lookup table from simulation config");
            dragCalc = new LookupTableDragCalculator(simOptions.getDragLookupTable());
        } else {
            dragCalc = new BarrowmanDragCalculator();
        }

        if (simOptions.hasStabilityLookup()) {
            logger.info("Aero model: auto -> using stability lookup table from simulation config");
            stabilityCalc = new LookupTableStabilityCalculator(simOptions.getStabilityLookupTable());
        } else {
            stabilityCalc = new BarrowmanStabilityCalculator();
        }

        // If both are Barrowman, use the simpler constructor
        if (dragCalc instanceof BarrowmanDragCalculator && stabilityCalc instanceof BarrowmanStabilityCalculator) {
            logger.info("Aero model: auto -> Barrowman (analytical, no lookups configured)");
            return new BarrowmanCalculator();
        }

        return new BarrowmanCalculator(stabilityCalc, dragCalc);
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
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    // --- Step 3: Extract Metadata ---

    /**
     * Extract metadata into a top-level 'meta' struct.
     * Contains version info, file hash, coordinate conventions, and export settings.
     */
    private void extractMetadata(FlightConfiguration config, Simulation sim,
                                  SimulationOptions simOptions,
                                  AerodynamicCalculator aeroCalc,
                                  MatFile matFile) {
        Struct meta = Mat5.newStruct();

        // Version info
        String orVersion;
        try {
            orVersion = BuildProperties.getVersion();
        } catch (Exception e) {
            orVersion = "unknown";
        }
        meta.set("openrocket_version", Mat5.newString(orVersion));
        meta.set("exporter_version", Mat5.newString(EXPORTER_VERSION));

        // File info
        meta.set("ork_file", Mat5.newString(orkFile.getAbsolutePath()));
        meta.set("ork_sha256", Mat5.newString(computeFileHash(orkFile)));
        meta.set("export_timestamp", Mat5.newString(Instant.now().toString()));

        // Simulation info
        meta.set("sim_index", Mat5.newScalar(simIndex));
        meta.set("sim_name", Mat5.newString(sim.getName()));
        meta.set("flight_config", Mat5.newString(config.getName()));

        // Aero model info
        String dragModelName = (aeroCalc instanceof BarrowmanCalculator) ? "barrowman" : aeroModel;
        meta.set("aero_model", Mat5.newString(dragModelName));

        // Export settings
        meta.set("ref_altitude_m", Mat5.newScalar(refAltitude));
        meta.set("alt_max_m", Mat5.newScalar(altMax));
        meta.set("alt_step_m", Mat5.newScalar(altStep));
        meta.set("mach_max", Mat5.newScalar(machMax));
        meta.set("mach_step", Mat5.newScalar(machStep));
        meta.set("aoa_max_deg", Mat5.newScalar(aoaMaxDeg));
        meta.set("aoa_step_deg", Mat5.newScalar(aoaStepDeg));
        meta.set("mass_dt", Mat5.newScalar(massDt));

        // Motor info
        Collection<MotorConfiguration> motorConfigs = config.getActiveMotors();
        int nMotors = motorConfigs.size();
        StringBuilder motorNames = new StringBuilder();
        for (MotorConfiguration mc : motorConfigs) {
            if (motorNames.length() > 0) motorNames.append(";");
            motorNames.append(mc.getMotor().getDesignation());
        }
        meta.set("motors", Mat5.newString(motorNames.toString()));
        meta.set("n_motors", Mat5.newScalar(nMotors));
        meta.set("n_stages", Mat5.newScalar(config.getStageCount()));

        // Motor identity in metadata (v2.1)
        if (!motorConfigs.isEmpty()) {
            MotorConfiguration firstMC = motorConfigs.iterator().next();
            Motor mot = firstMC.getMotor();
            if (mot instanceof ThrustCurveMotor tcm) {
                meta.set("motor_manufacturer", Mat5.newString(tcm.getManufacturer().getDisplayName()));
                meta.set("motor_type", Mat5.newString(tcm.getMotorType().name()));
            }
        }

        // Coordinate frame conventions
        meta.set("coord_origin", Mat5.newString("nose_tip"));
        meta.set("coord_x_direction", Mat5.newString("positive_aft"));
        meta.set("coord_frame", Mat5.newString("body_fixed"));
        meta.set("inertia_reference", Mat5.newString("about_combined_CG"));
        meta.set("inertia_frame", Mat5.newString("body_axes_Ixx_roll_Iyy_pitch_Izz_yaw"));
        meta.set("inertia_tensor_format", Mat5.newString("3x3xN_diagonal_only_off_diag_zero"));

        // Aerospace Blockset mapping (v2.1)
        meta.set("or_body_frame", Mat5.newString("x_aft_y_right_z_up"));
        meta.set("aero_body_frame", Mat5.newString("x_fwd_y_right_z_down"));
        meta.set("or_to_aero_transform", Mat5.newString("diag([-1, 1, -1])"));
        meta.set("aero_world_frame", Mat5.newString("NED"));
        meta.set("or_world_frame", Mat5.newString("z_up"));

        // Coefficient definitions
        meta.set("cd_definition", Mat5.newString("total_drag_coeff_CD=D/(q*Sref)"));
        meta.set("cn_definition", Mat5.newString("normal_force_coeff_CN=N/(q*Sref)"));
        meta.set("cm_definition", Mat5.newString("pitching_moment_coeff_about_nose_Cm=M/(q*Sref*Lref)"));
        meta.set("aoa_definition", Mat5.newString("angle_of_attack_radians_from_velocity_to_body_x"));

        // Units and notes
        meta.set("units", Mat5.newString("SI: m, kg, s, rad, N, Pa, K"));
        meta.set("re_note", Mat5.newString(
                "Friction drag depends on altitude via Reynolds number. " +
                "3D tables span 0 to " + altMax + " m in " + altStep + " m steps."));
        meta.set("coord_note", Mat5.newString(
                "cg_motor_x_vs_time uses Motor.getCMx(t) which is CG relative to motor front, " +
                "NOT in rocket frame. cg_x_vs_time is in rocket body frame (nose=0)."));

        matFile.addArray("meta", meta);
    }

    // --- Step 4: Extract Motor Data ---

    /**
     * Extract motor thrust curve and metadata with monotonic time guarantee and burnout tail.
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
            matFile.addArray("motor_propellant_mass", Mat5.newScalar(0));
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
            timePoints = tcm.getTimePoints().clone();
            thrustPoints = tcm.getThrustPoints().clone();
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

        // Make time strictly monotonic
        makeStrictlyMonotonic(timePoints, thrustPoints);

        // Scale thrust by number of motors
        double[] totalThrust = new double[thrustPoints.length];
        for (int i = 0; i < thrustPoints.length; i++) {
            totalThrust[i] = thrustPoints[i] * nMotors;
        }

        // Append burnout tail: if last thrust > 1e-6, add zero crossing
        int len = timePoints.length;
        if (totalThrust[len - 1] > 1e-6) {
            timePoints = Arrays.copyOf(timePoints, len + 1);
            totalThrust = Arrays.copyOf(totalThrust, len + 1);
            timePoints[len] = timePoints[len - 1] + 0.001;
            totalThrust[len] = 0.0;
            len++;
        }

        // Add far-future hold point for safe Simulink LUT clipping
        timePoints = Arrays.copyOf(timePoints, len + 1);
        totalThrust = Arrays.copyOf(totalThrust, len + 1);
        timePoints[len] = timePoints[len - 1] + 100.0;
        totalThrust[len] = 0.0;

        // Propellant mass = (launchMass - burnoutMass) * nMotors
        double propellantMass = (motor.getLaunchMass() - motor.getBurnoutMass()) * nMotors;

        // Compute total impulse via trapezoidal integration of the (scaled) thrust curve
        double computedTotalImpulse = 0.0;
        for (int i = 1; i < timePoints.length; i++) {
            double dt = timePoints[i] - timePoints[i - 1];
            computedTotalImpulse += 0.5 * (totalThrust[i - 1] + totalThrust[i]) * dt;
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
        matFile.addArray("motor_propellant_mass", Mat5.newScalar(propellantMass));
        matFile.addArray("motor_total_impulse", Mat5.newScalar(computedTotalImpulse));

        // Motor identity (v2.1)
        if (motor instanceof ThrustCurveMotor tcm) {
            matFile.addArray("motor_manufacturer", Mat5.newString(tcm.getManufacturer().getDisplayName()));
            matFile.addArray("motor_avg_thrust", Mat5.newScalar(tcm.getAverageThrustEstimate()));
            matFile.addArray("motor_max_thrust", Mat5.newScalar(tcm.getMaxThrustEstimate()));
            matFile.addArray("motor_impulse_class", Mat5.newString(tcm.getMotorType().name()));
        } else {
            matFile.addArray("motor_manufacturer", Mat5.newString("unknown"));
            matFile.addArray("motor_avg_thrust", Mat5.newScalar(0));
            matFile.addArray("motor_max_thrust", Mat5.newScalar(0));
            matFile.addArray("motor_impulse_class", Mat5.newString("unknown"));
        }
    }

    // --- Step 5: Extract Mass Data ---

    /**
     * Extract time-varying mass, inertia, and CG data with separated components,
     * proper parallel-axis-theorem, and dI/dt arrays.
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

        // -- Static mass/inertia values (legacy + new separated) --

        // Legacy aliases (backward compat)
        matFile.addArray("mass_launch", Mat5.newScalar(launchMass));
        matFile.addArray("mass_burnout", Mat5.newScalar(burnoutMass));
        matFile.addArray("mass_structure", Mat5.newScalar(structureMass));

        matFile.addArray("Ixx_launch", Mat5.newScalar(launchBody.getIxx()));
        matFile.addArray("Iyy_launch", Mat5.newScalar(launchBody.getIyy()));
        matFile.addArray("Izz_launch", Mat5.newScalar(launchBody.getIzz()));
        matFile.addArray("cg_launch_x", Mat5.newScalar(launchBody.getCM().getX()));

        // New separated breakdown: structure
        matFile.addArray("m_structure", Mat5.newScalar(structureMass));
        matFile.addArray("cg_structure_x", Mat5.newScalar(structure.getCM().getX()));
        matFile.addArray("I_structure_xx", Mat5.newScalar(structure.getIxx()));
        matFile.addArray("I_structure_yy", Mat5.newScalar(structure.getIyy()));
        matFile.addArray("I_structure_zz", Mat5.newScalar(structure.getIzz()));

        // New separated breakdown: motor at launch
        RigidBody motorLaunch = computeMotorRigidBody(config, 0.0);
        matFile.addArray("m_motor_launch", Mat5.newScalar(motorLaunch.getMass()));
        matFile.addArray("cg_motor_launch_x", Mat5.newScalar(motorLaunch.getCM().getX()));
        matFile.addArray("I_motor_launch_xx", Mat5.newScalar(motorLaunch.getIxx()));
        matFile.addArray("I_motor_launch_yy", Mat5.newScalar(motorLaunch.getIyy()));
        matFile.addArray("I_motor_launch_zz", Mat5.newScalar(motorLaunch.getIzz()));

        // New separated breakdown: total
        matFile.addArray("m_total_launch", Mat5.newScalar(launchMass));
        matFile.addArray("m_total_burnout", Mat5.newScalar(burnoutMass));
        matFile.addArray("I_total_launch_xx", Mat5.newScalar(launchBody.getIxx()));
        matFile.addArray("I_total_launch_yy", Mat5.newScalar(launchBody.getIyy()));
        matFile.addArray("I_total_launch_zz", Mat5.newScalar(launchBody.getIzz()));

        // -- Time-varying mass from motor burn --
        Collection<MotorConfiguration> motorConfigs = config.getActiveMotors();
        if (motorConfigs.isEmpty()) {
            // No motors - constant mass
            matFile.addArray("mass_time", toRowVector(new double[]{0}));
            matFile.addArray("mass_total", toRowVector(new double[]{structureMass}));
            matFile.addArray("mass_propellant", toRowVector(new double[]{0}));
            matFile.addArray("Ixx_vs_time", toRowVector(new double[]{structure.getIxx()}));
            matFile.addArray("Iyy_vs_time", toRowVector(new double[]{structure.getIyy()}));
            matFile.addArray("Izz_vs_time", toRowVector(new double[]{structure.getIzz()}));
            matFile.addArray("cg_x_vs_time", toRowVector(new double[]{structure.getCM().getX()}));
            matFile.addArray("dIxx_dt", toRowVector(new double[]{0}));
            matFile.addArray("dIyy_dt", toRowVector(new double[]{0}));
            matFile.addArray("dIzz_dt", toRowVector(new double[]{0}));
            matFile.addArray("m_motor_vs_time", toRowVector(new double[]{0}));
            matFile.addArray("cg_motor_x_vs_time", toRowVector(new double[]{0}));
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
        double[] motorMassTime = new double[nSteps];
        double[] motorCgxTime = new double[nSteps];

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
            motorMassTime[i] = motorMassTotal;
            motorCgxTime[i] = (motorMassTotal > 0) ? (motorMassCgXWeighted / motorMassTotal) : 0;

            // Combined CG (1D along x-axis): weighted average
            double totalMass = massTotal[i];
            if (totalMass > 0) {
                cgxTime[i] = (structureMass * structure.getCM().getX() + motorMassCgXWeighted) / totalMass;
            } else {
                cgxTime[i] = structure.getCM().getX();
            }

            // Combined inertia using parallel axis theorem
            // Structure contribution: structure inertia + PAT offset from combined CG
            double dx_struct = structure.getCM().getX() - cgxTime[i];
            // Motor contribution: motor inertia + PAT offset from combined CG
            double motorCgX = (motorMassTotal > 0) ? (motorMassCgXWeighted / motorMassTotal) : cgxTime[i];
            double dx_motor = motorCgX - cgxTime[i];

            ixxTime[i] = structure.getIxx() + motorIxx; // Axial: no parallel axis offset
            // FIX: include motor PAT term (was missing in v1.0)
            iyyTime[i] = structure.getIyy() + structureMass * dx_struct * dx_struct
                    + motorIyy + motorMassTotal * dx_motor * dx_motor;
            izzTime[i] = structure.getIzz() + structureMass * dx_struct * dx_struct
                    + motorIzz + motorMassTotal * dx_motor * dx_motor;
        }

        // Ensure mass_time is strictly monotonic
        makeStrictlyMonotonicTime(massTime);

        // Compute dI/dt using central finite differences
        double[] dIxxDt = centralFiniteDifference(massTime, ixxTime);
        double[] dIyyDt = centralFiniteDifference(massTime, iyyTime);
        double[] dIzzDt = centralFiniteDifference(massTime, izzTime);

        matFile.addArray("mass_time", toRowVector(massTime));
        matFile.addArray("mass_total", toRowVector(massTotal));
        matFile.addArray("mass_propellant", toRowVector(massPropellant));
        matFile.addArray("Ixx_vs_time", toRowVector(ixxTime));
        matFile.addArray("Iyy_vs_time", toRowVector(iyyTime));
        matFile.addArray("Izz_vs_time", toRowVector(izzTime));
        matFile.addArray("cg_x_vs_time", toRowVector(cgxTime));
        matFile.addArray("dIxx_dt", toRowVector(dIxxDt));
        matFile.addArray("dIyy_dt", toRowVector(dIyyDt));
        matFile.addArray("dIzz_dt", toRowVector(dIzzDt));
        matFile.addArray("m_motor_vs_time", toRowVector(motorMassTime));
        matFile.addArray("cg_motor_x_vs_time", toRowVector(motorCgxTime));

        // 3x3 inertia tensor vs time (v2.1) — diagonal only, zeros on off-diag
        // Shape: 3 x 3 x nSteps
        Matrix iTensor = Mat5.newMatrix(new int[]{3, 3, nSteps});
        for (int i = 0; i < nSteps; i++) {
            iTensor.setDouble(new int[]{0, 0, i}, ixxTime[i]);
            iTensor.setDouble(new int[]{1, 1, i}, iyyTime[i]);
            iTensor.setDouble(new int[]{2, 2, i}, izzTime[i]);
            // off-diagonals remain 0 (OR only computes diagonal MOI)
        }
        matFile.addArray("I_tensor_vs_time", iTensor);

        // Rocket-prefixed invariant mass (v2.1 partitioning)
        matFile.addArray("rocket_mass_structure", Mat5.newScalar(structureMass));
    }

    /**
     * Compute motor RigidBody at a given time (for separated breakdown).
     */
    private RigidBody computeMotorRigidBody(FlightConfiguration config, double time) {
        Collection<MotorConfiguration> motorConfigs = config.getActiveMotors();
        double motorMass = 0;
        double motorCgXWeighted = 0;
        double motorIxx = 0, motorIyy = 0, motorIzz = 0;
        for (MotorConfiguration mc : motorConfigs) {
            Motor m = mc.getMotor();
            double mMass = m.getTotalMass(time);
            double mCGx = m.getCMx(time);
            motorMass += mMass;
            motorCgXWeighted += mMass * mCGx;
            motorIxx += m.getUnitIxx() * mMass;
            motorIyy += m.getUnitIyy() * mMass;
            motorIzz += m.getUnitIzz() * mMass;
        }
        double cgX = (motorMass > 0) ? (motorCgXWeighted / motorMass) : 0;
        // Use a simple coordinate - weight carries the mass
        CoordinateIF cm = new info.openrocket.core.util.Coordinate(cgX, 0, 0, motorMass);
        return new RigidBody(cm, motorIxx, motorIyy, motorIzz);
    }

    // --- Step 6: Extract Aero Data ---

    /**
     * Extract 3D aerodynamic coefficient tables: CD, CN, Cm, Croll, CrollDamp, CrollForce vs Mach × AoA × Altitude.
     * Altitude dimension captures Reynolds-number and density changes via ExtendedISAModel.
     */
    private void extractAeroData(FlightConfiguration config, AerodynamicCalculator calc, MatFile matFile) {
        ExtendedISAModel atm = new ExtendedISAModel();

        // Build breakpoint arrays
        int nMach = (int) Math.round(machMax / machStep) + 1;
        int nAoA = (int) Math.round(aoaMaxDeg / aoaStepDeg) + 1;
        int nAlt = (int) Math.round(altMax / altStep) + 1;
        if (nAlt < 1) nAlt = 1;

        double[] machBp = new double[nMach];
        double[] aoaBp = new double[nAoA]; // in radians
        double[] altBp = new double[nAlt];

        for (int i = 0; i < nMach; i++) {
            machBp[i] = i * machStep;
        }
        for (int j = 0; j < nAoA; j++) {
            aoaBp[j] = Math.toRadians(j * aoaStepDeg);
        }
        for (int k = 0; k < nAlt; k++) {
            altBp[k] = k * altStep;
        }

        // Avoid Mach = 0 exactly (can cause division issues in some calcs)
        if (machBp[0] < 1e-6) {
            machBp[0] = 0.001;
        }

        long totalEvals = (long) nMach * nAoA * nAlt;
        logger.info("Computing 3D aero tables: {} Mach x {} AoA x {} Alt = {} evaluations",
                nMach, nAoA, nAlt, totalEvals);

        // Allocate 3D tables (machIdx x aoaIdx x altIdx)
        double[][][] cdTable = new double[nMach][nAoA][nAlt];
        double[][][] cnTable = new double[nMach][nAoA][nAlt];
        double[][][] cmTable = new double[nMach][nAoA][nAlt];
        double[][][] crollTable = new double[nMach][nAoA][nAlt];
        double[][][] cdAxialTable = new double[nMach][nAoA][nAlt];
        double[][][] pressureCdTable = new double[nMach][nAoA][nAlt];
        double[][][] frictionCdTable = new double[nMach][nAoA][nAlt];
        double[][][] baseCdTable = new double[nMach][nAoA][nAlt];
        double[][][] crollDampTable = new double[nMach][nAoA][nAlt];
        double[][][] crollForceTable = new double[nMach][nAoA][nAlt];

        WarningSet aeroWarnings = new WarningSet();

        for (int i = 0; i < nMach; i++) {
            for (int j = 0; j < nAoA; j++) {
                for (int k = 0; k < nAlt; k++) {
                    AtmosphericConditions atmCond = atm.getConditions(altBp[k]);

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
                        cdTable[i][j][k] = forces.getCD();
                        cnTable[i][j][k] = forces.getCN();
                        cmTable[i][j][k] = forces.getCm();
                        crollTable[i][j][k] = forces.getCroll();
                        cdAxialTable[i][j][k] = forces.getCDaxial();
                        pressureCdTable[i][j][k] = forces.getPressureCD();
                        frictionCdTable[i][j][k] = forces.getFrictionCD();
                        baseCdTable[i][j][k] = forces.getBaseCD();
                        crollDampTable[i][j][k] = forces.getCrollDamp();
                        crollForceTable[i][j][k] = forces.getCrollForce();
                    } catch (Exception e) {
                        logger.warn("Aero calc failed at Mach={}, AoA={}deg, Alt={}m: {}",
                                machBp[i], Math.toDegrees(aoaBp[j]), altBp[k], e.getMessage());
                        cdTable[i][j][k] = Double.NaN;
                        cnTable[i][j][k] = Double.NaN;
                        cmTable[i][j][k] = Double.NaN;
                        crollTable[i][j][k] = Double.NaN;
                        cdAxialTable[i][j][k] = Double.NaN;
                        crollDampTable[i][j][k] = Double.NaN;
                        crollForceTable[i][j][k] = Double.NaN;
                    }
                }
            }
            // Progress
            if (i > 0 && i % 50 == 0) {
                logger.info("  Aero progress: {}/{} Mach points", i, nMach);
            }
        }

        matFile.addArray("mach_bp", toRowVector(machBp));
        matFile.addArray("aoa_bp", toRowVector(aoaBp));
        matFile.addArray("alt_bp", toRowVector(altBp));
        matFile.addArray("CD_table", to3DMatrix(cdTable));
        matFile.addArray("CN_table", to3DMatrix(cnTable));
        matFile.addArray("Cm_table", to3DMatrix(cmTable));
        matFile.addArray("Croll_table", to3DMatrix(crollTable));
        matFile.addArray("CDaxial_table", to3DMatrix(cdAxialTable));
        matFile.addArray("pressureCD_table", to3DMatrix(pressureCdTable));
        matFile.addArray("frictionCD_table", to3DMatrix(frictionCdTable));
        matFile.addArray("baseCD_table", to3DMatrix(baseCdTable));
        matFile.addArray("CrollDamp_table", to3DMatrix(crollDampTable));
        matFile.addArray("CrollForce_table", to3DMatrix(crollForceTable));
    }

    // --- Step 7: Extract Geometry (all fin sets) ---

    /**
     * Extract reference geometry and per-finset data from the rocket.
     * Exports fin0_*, fin1_*, etc. for each FinSet found.
     */
    private void extractGeometry(FlightConfiguration config, Rocket rocket, MatFile matFile) {
        double refLength = config.getReferenceLength();
        double refArea = config.getReferenceArea();

        matFile.addArray("ref_length", Mat5.newScalar(refLength));
        matFile.addArray("ref_area", Mat5.newScalar(refArea));

        // Compute total rocket length by iterating the component tree
        double totalLength = 0;

        // Collect all fin sets
        List<FinSet> allFinSets = new ArrayList<>();

        for (RocketComponent comp : rocket) {
            if (comp instanceof AxialStage || comp instanceof BodyTube ||
                    comp instanceof NoseCone || comp instanceof Transition) {
                totalLength += comp.getLength();
            }
            if (comp instanceof FinSet fs) {
                allFinSets.add(fs);
            }
        }

        matFile.addArray("rocket_length", Mat5.newScalar(totalLength));

        // Legacy single-finset fields for backward compat (use last fin set found, or 0)
        int finCount = 0;
        double finRootChord = 0, finSpan = 0, finSweep = 0;
        if (!allFinSets.isEmpty()) {
            FinSet lastFin = allFinSets.get(allFinSets.size() - 1);
            finCount = lastFin.getFinCount();
            finSpan = lastFin.getSpan();
            if (lastFin instanceof TrapezoidFinSet tfs) {
                finRootChord = tfs.getRootChord();
                finSweep = tfs.getSweep();
            } else if (lastFin instanceof EllipticalFinSet efs) {
                finRootChord = efs.getLength();
            }
        }
        matFile.addArray("fin_count", Mat5.newScalar(finCount));
        matFile.addArray("fin_root_chord", Mat5.newScalar(finRootChord));
        matFile.addArray("fin_span", Mat5.newScalar(finSpan));
        matFile.addArray("fin_sweep", Mat5.newScalar(finSweep));

        // Per-finset export: fin0_*, fin1_*, ...
        for (int idx = 0; idx < allFinSets.size(); idx++) {
            String prefix = "fin" + idx + "_";
            FinSet fs = allFinSets.get(idx);

            matFile.addArray(prefix + "count", Mat5.newScalar(fs.getFinCount()));
            matFile.addArray(prefix + "span", Mat5.newScalar(fs.getSpan()));
            matFile.addArray(prefix + "cant_angle_rad", Mat5.newScalar(fs.getCantAngle()));
            matFile.addArray(prefix + "thickness", Mat5.newScalar(fs.getThickness()));

            // Position along rocket (x from nose)
            CoordinateIF[] locs = fs.getComponentLocations();
            if (locs.length > 0) {
                matFile.addArray(prefix + "position_x", Mat5.newScalar(locs[0].getX()));
            }

            if (fs instanceof TrapezoidFinSet tfs) {
                matFile.addArray(prefix + "root_chord", Mat5.newScalar(tfs.getRootChord()));
                matFile.addArray(prefix + "tip_chord", Mat5.newScalar(tfs.getTipChord()));
                matFile.addArray(prefix + "sweep", Mat5.newScalar(tfs.getSweep()));
                matFile.addArray(prefix + "type", Mat5.newString("trapezoid"));
            } else if (fs instanceof EllipticalFinSet efs) {
                matFile.addArray(prefix + "root_chord", Mat5.newScalar(efs.getLength()));
                matFile.addArray(prefix + "type", Mat5.newString("elliptical"));
            } else {
                matFile.addArray(prefix + "type", Mat5.newString("freeform"));
            }
        }

        matFile.addArray("n_finsets", Mat5.newScalar(allFinSets.size()));

        // Rocket-prefixed invariant aliases (v2.1 partitioning)
        matFile.addArray("rocket_ref_length", Mat5.newScalar(refLength));
        matFile.addArray("rocket_ref_area", Mat5.newScalar(refArea));
        matFile.addArray("rocket_length", Mat5.newScalar(totalLength));
        matFile.addArray("rocket_n_finsets", Mat5.newScalar(allFinSets.size()));
        matFile.addArray("rocket_mass_structure", Mat5.newScalar(0)); // placeholder, filled by extractMassData
        matFile.addArray("rocket_fin_count", Mat5.newScalar(finCount));
    }

    /**
     * Extract simulation settings (launch rod, atmosphere, wind, scenario data).
     */
    private void extractSimSettings(SimulationOptions opts, MatFile matFile) {
        // Launch rod
        matFile.addArray("launch_rod_length", Mat5.newScalar(opts.getLaunchRodLength()));
        matFile.addArray("launch_rod_angle", Mat5.newScalar(opts.getLaunchRodAngle()));
        matFile.addArray("launch_rod_direction", Mat5.newScalar(opts.getLaunchRodDirection()));

        // Launch site
        matFile.addArray("launch_altitude", Mat5.newScalar(opts.getLaunchAltitude()));
        matFile.addArray("launch_latitude", Mat5.newScalar(opts.getLaunchLatitude()));
        matFile.addArray("launch_longitude", Mat5.newScalar(opts.getLaunchLongitude()));

        // Atmosphere
        matFile.addArray("launch_temperature", Mat5.newScalar(opts.getLaunchTemperature()));
        matFile.addArray("launch_pressure", Mat5.newScalar(opts.getLaunchPressure()));
        matFile.addArray("isa_atmosphere", Mat5.newScalar(opts.isISAAtmosphere() ? 1.0 : 0.0));
        matFile.addArray("launch_relative_humidity", Mat5.newScalar(opts.getLaunchRelativeHumidity()));

        // Time step
        matFile.addArray("time_step", Mat5.newScalar(opts.getTimeStep()));

        // Wind (v2.1 scenario data)
        PinkNoiseWindModel windModel = opts.getAverageWindModel();
        matFile.addArray("wind_speed_avg", Mat5.newScalar(windModel.getAverage()));
        matFile.addArray("wind_direction", Mat5.newScalar(windModel.getDirection()));
        matFile.addArray("wind_turbulence_intensity", Mat5.newScalar(windModel.getTurbulenceIntensity()));
        matFile.addArray("launch_into_wind", Mat5.newScalar(opts.getLaunchIntoWind() ? 1.0 : 0.0));
        matFile.addArray("wind_model_type", Mat5.newString(opts.getWindModelType().name()));

        // Scenario-prefixed aliases (v2.1 partitioning)
        matFile.addArray("scenario_wind_speed", Mat5.newScalar(windModel.getAverage()));
        matFile.addArray("scenario_wind_direction", Mat5.newScalar(windModel.getDirection()));
        matFile.addArray("scenario_wind_turbulence", Mat5.newScalar(windModel.getTurbulenceIntensity()));
        matFile.addArray("scenario_launch_altitude", Mat5.newScalar(opts.getLaunchAltitude()));
        matFile.addArray("scenario_launch_temperature", Mat5.newScalar(opts.getLaunchTemperature()));
        matFile.addArray("scenario_launch_pressure", Mat5.newScalar(opts.getLaunchPressure()));
        matFile.addArray("scenario_isa_atmosphere", Mat5.newScalar(opts.isISAAtmosphere() ? 1.0 : 0.0));
        matFile.addArray("scenario_launch_rod_length", Mat5.newScalar(opts.getLaunchRodLength()));
        matFile.addArray("scenario_launch_rod_angle", Mat5.newScalar(opts.getLaunchRodAngle()));
        matFile.addArray("scenario_time_step", Mat5.newScalar(opts.getTimeStep()));
    }

    // --- Step 8: Extract Sim Results ---

    /**
     * Run the OpenRocket simulation and extract results for validation,
     * including additional channels: AoA, roll rate, CG, CP, stability.
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

            // Rail exit data (v2.1)
            matFile.addArray("or_rail_exit_velocity", Mat5.newScalar(safeGet(data::getLaunchRodVelocity)));

            // Time-series from the first branch (sustainer)
            if (data.getBranchCount() > 0) {
                FlightDataBranch branch = data.getBranch(0);

                // Original channels
                addTimeSeries(matFile, "or_time", branch, FlightDataType.TYPE_TIME);
                addTimeSeries(matFile, "or_altitude", branch, FlightDataType.TYPE_ALTITUDE);
                addTimeSeries(matFile, "or_velocity", branch, FlightDataType.TYPE_VELOCITY_TOTAL);
                addTimeSeries(matFile, "or_acceleration", branch, FlightDataType.TYPE_ACCELERATION_TOTAL);
                addTimeSeries(matFile, "or_mach", branch, FlightDataType.TYPE_MACH_NUMBER);
                addTimeSeries(matFile, "or_mass", branch, FlightDataType.TYPE_MASS);
                addTimeSeries(matFile, "or_thrust", branch, FlightDataType.TYPE_THRUST_FORCE);
                addTimeSeries(matFile, "or_drag", branch, FlightDataType.TYPE_DRAG_FORCE);
                addTimeSeries(matFile, "or_cd", branch, FlightDataType.TYPE_DRAG_COEFF);

                // New channels (step 8)
                addTimeSeries(matFile, "or_aoa", branch, FlightDataType.TYPE_AOA);
                addTimeSeries(matFile, "or_roll_rate", branch, FlightDataType.TYPE_ROLL_RATE);
                addTimeSeries(matFile, "or_cg", branch, FlightDataType.TYPE_CG_LOCATION);
                addTimeSeries(matFile, "or_cp", branch, FlightDataType.TYPE_CP_LOCATION);
                addTimeSeries(matFile, "or_stability", branch, FlightDataType.TYPE_STABILITY);
            }
            logger.info("OpenRocket simulation complete. Max altitude: {} m", safeGet(data::getMaxAltitude));
        } catch (Exception e) {
            logger.error("OpenRocket simulation failed: {}", e.getMessage(), e);
        }
    }

    /**
     * Helper to add a time series from a FlightDataBranch to the MatFile.
     */
    private void addTimeSeries(MatFile matFile, String varName, FlightDataBranch branch, FlightDataType type) {
        try {
            List<Double> values = branch.get(type);
            if (values != null && !values.isEmpty()) {
                matFile.addArray(varName, toRowVectorFromList(values));
            }
        } catch (Exception e) {
            logger.debug("Could not extract {}: {}", varName, e.getMessage());
        }
    }

    // --- Step 9: Utility Methods ---

    /**
     * Make time and value arrays strictly monotonic by nudging duplicate time points.
     * If time[i] <= time[i-1], set time[i] = time[i-1] + epsilon.
     */
    private void makeStrictlyMonotonic(double[] time, double[] values) {
        double epsilon = 1e-9;
        for (int i = 1; i < time.length; i++) {
            if (time[i] <= time[i - 1]) {
                time[i] = time[i - 1] + epsilon;
            }
        }
    }

    /**
     * Make a time array strictly monotonic (in-place), without value array.
     */
    private void makeStrictlyMonotonicTime(double[] time) {
        double epsilon = 1e-9;
        for (int i = 1; i < time.length; i++) {
            if (time[i] <= time[i - 1]) {
                time[i] = time[i - 1] + epsilon;
            }
        }
    }

    /**
     * Compute central finite differences: dy/dx.
     * Uses forward difference at first point, backward at last, central elsewhere.
     */
    private double[] centralFiniteDifference(double[] x, double[] y) {
        int n = x.length;
        double[] dydx = new double[n];
        if (n < 2) {
            return dydx; // all zeros
        }

        // Forward difference at first point
        dydx[0] = (y[1] - y[0]) / Math.max(x[1] - x[0], 1e-12);

        // Central difference in the middle
        for (int i = 1; i < n - 1; i++) {
            double dt = x[i + 1] - x[i - 1];
            if (dt > 1e-12) {
                dydx[i] = (y[i + 1] - y[i - 1]) / dt;
            }
        }

        // Backward difference at last point
        dydx[n - 1] = (y[n - 1] - y[n - 2]) / Math.max(x[n - 1] - x[n - 2], 1e-12);

        return dydx;
    }

    /**
     * Compute SHA-256 hash of a file, return hex string.
     */
    private String computeFileHash(File file) {
        try {
            MessageDigest digest = MessageDigest.getInstance("SHA-256");
            try (FileInputStream fis = new FileInputStream(file)) {
                byte[] buffer = new byte[8192];
                int bytesRead;
                while ((bytesRead = fis.read(buffer)) != -1) {
                    digest.update(buffer, 0, bytesRead);
                }
            }
            byte[] hash = digest.digest();
            StringBuilder sb = new StringBuilder();
            for (byte b : hash) {
                sb.append(String.format("%02x", b));
            }
            return sb.toString();
        } catch (Exception e) {
            logger.warn("Could not compute file hash: {}", e.getMessage());
            return "unknown";
        }
    }

    /**
     * Convert a double[] to a 1xN MATLAB row vector.
     */
    private Matrix toRowVector(double[] data) {
        Matrix m = Mat5.newMatrix(1, data.length);
        for (int i = 0; i < data.length; i++) {
            m.setDouble(0, i, data[i]);
        }
        return m;
    }

    /**
     * Convert a List of Double to a 1xN MATLAB row vector.
     */
    private Matrix toRowVectorFromList(List<Double> data) {
        Matrix m = Mat5.newMatrix(1, data.size());
        for (int i = 0; i < data.size(); i++) {
            m.setDouble(0, i, data.get(i));
        }
        return m;
    }

    /**
     * Convert a double[][] (nRows x nCols) to a MATLAB matrix.
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
     * Convert a double[][][] (dim1 x dim2 x dim3) to a 3D MATLAB matrix.
     * MATLAB stores column-major. We use setDouble(int[]) for N-D indexing.
     */
    private Matrix to3DMatrix(double[][][] data) {
        int d1 = data.length;
        int d2 = data[0].length;
        int d3 = data[0][0].length;
        Matrix m = Mat5.newMatrix(new int[]{d1, d2, d3});
        for (int i = 0; i < d1; i++) {
            for (int j = 0; j < d2; j++) {
                for (int k = 0; k < d3; k++) {
                    m.setDouble(new int[]{i, j, k}, data[i][j][k]);
                }
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
