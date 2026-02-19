package info.openrocket.exporter;

import org.junit.jupiter.api.*;
import us.hebi.matlab.mat.format.Mat5;
import us.hebi.matlab.mat.format.Mat5File;
import us.hebi.matlab.mat.types.MatFile;
import us.hebi.matlab.mat.types.Matrix;
import us.hebi.matlab.mat.types.Char;
import us.hebi.matlab.mat.types.Struct;
import us.hebi.matlab.mat.types.Sources;

import java.io.File;
import java.nio.file.Files;
import java.nio.file.Path;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Integration tests for the OrkExporter CLI tool.
 * <p>
 * These tests exercise the full pipeline: load .ork → extract data → write .mat → validate contents.
 * They use the example .ork files bundled with OpenRocket core.
 */
@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
class OrkExporterTest {

    private static String EXAMPLES_DIR;
    private static Path tempDir;

    @BeforeAll
    static void setup() throws Exception {
        tempDir = Files.createTempDirectory("ork-exporter-test");
        // Resolve examples directory relative to project root (parent of exporter/)
        // The working directory during tests is the subproject directory (exporter/),
        // so we go up one level to reach the project root.
        Path projectRoot = Path.of(System.getProperty("user.dir")).getParent();
        EXAMPLES_DIR = projectRoot.resolve("core/src/main/resources/datafiles/examples").toString() + File.separator;
    }

    @AfterAll
    static void cleanup() throws Exception {
        // Clean up temp files
        if (tempDir != null) {
            Files.walk(tempDir)
                    .sorted(java.util.Comparator.reverseOrder())
                    .map(Path::toFile)
                    .forEach(File::delete);
        }
    }

    // ─── Helper to run the exporter and return the exit code ─────────

    private int runExporter(String orkPath, String outPath, String... extraArgs) {
        // Build argument list
        var args = new java.util.ArrayList<String>();
        args.add("--ork");
        args.add(orkPath);
        args.add("--out");
        args.add(outPath);
        // Use coarse steps for fast tests
        args.add("--mach-step");
        args.add("0.5");
        args.add("--aoa-step");
        args.add("5.0");
        args.add("--alt-step");
        args.add("2500.0");
        args.addAll(java.util.Arrays.asList(extraArgs));

        return new picocli.CommandLine(new OrkExporter()).execute(args.toArray(new String[0]));
    }

    private Mat5File loadMat(String path) throws Exception {
        return Mat5.newReader(Sources.openFile(path)).readMat();
    }

    private double getScalar(Mat5File mat, String name) {
        Matrix m = mat.getMatrix(name);
        assertNotNull(m, "Variable '" + name + "' not found in MAT file");
        return m.getDouble(0, 0);
    }

    private double[] getRow(Mat5File mat, String name) {
        Matrix m = mat.getMatrix(name);
        assertNotNull(m, "Variable '" + name + "' not found in MAT file");
        int cols = m.getNumCols();
        double[] row = new double[cols];
        for (int i = 0; i < cols; i++) {
            row[i] = m.getDouble(0, i);
        }
        return row;
    }

    // ─── Tests ───────────────────────────────────────────────────────

    @Test
    @Order(1)
    @DisplayName("Export simple model rocket — basic sanity checks")
    void testSimpleModelRocket() throws Exception {
        String orkFile = EXAMPLES_DIR + "A simple model rocket.ork";
        String matPath = tempDir.resolve("simple_rocket.mat").toString();

        // Verify the .ork file exists
        assertTrue(new File(orkFile).exists(), "Test .ork file must exist: " + orkFile);

        int exitCode = runExporter(orkFile, matPath);
        assertEquals(0, exitCode, "Exporter should succeed");

        // Verify MAT file was created
        File matFile = new File(matPath);
        assertTrue(matFile.exists(), "MAT file should be created");
        assertTrue(matFile.length() > 1000, "MAT file should have meaningful size");

        // Read and validate contents
        Mat5File mat = loadMat(matPath);

        // Mass properties
        double launchMass = getScalar(mat, "mass_launch");
        double burnoutMass = getScalar(mat, "mass_burnout");
        double structMass = getScalar(mat, "mass_structure");

        assertTrue(launchMass > 0, "Launch mass must be positive");
        assertTrue(burnoutMass > 0, "Burnout mass must be positive");
        assertTrue(structMass > 0, "Structure mass must be positive");
        assertTrue(launchMass > burnoutMass, "Launch mass must exceed burnout mass (has propellant)");
        assertTrue(launchMass > structMass, "Launch mass must exceed structure mass");
        assertTrue(burnoutMass >= structMass, "Burnout mass must be >= structure mass (includes motor casing)");

        // Motor properties
        double burnTime = getScalar(mat, "burn_time");
        assertTrue(burnTime > 0 && burnTime < 30, "Burn time should be reasonable (got " + burnTime + ")");
        assertEquals(1.0, getScalar(mat, "n_motors"), "Simple rocket should have 1 motor");

        // Thrust curve
        double[] thrustTime = getRow(mat, "thrust_time");
        double[] thrustForce = getRow(mat, "thrust_force");
        assertEquals(thrustTime.length, thrustForce.length, "Thrust time and force arrays must have same length");
        assertTrue(thrustTime.length >= 3, "Thrust curve should have multiple points");
        assertEquals(0.0, thrustTime[0], 1e-6, "Thrust curve should start at t=0");
        assertEquals(0.0, thrustForce[thrustForce.length - 1], 0.01, "Thrust curve should end at zero");

        // Verify thrust is non-negative
        for (int i = 0; i < thrustForce.length; i++) {
            assertTrue(thrustForce[i] >= 0, "Thrust must be non-negative at index " + i);
        }

        // Time-varying mass
        double[] massTime = getRow(mat, "mass_time");
        double[] massTotal = getRow(mat, "mass_total");
        assertEquals(massTime.length, massTotal.length, "Mass arrays must have same length");
        assertTrue(massTime.length >= 2, "Mass curve should have multiple points");

        // Mass should be monotonically decreasing during burn
        for (int i = 1; i < massTotal.length; i++) {
            assertTrue(massTotal[i] <= massTotal[i - 1] + 1e-9,
                    "Mass should decrease during burn at index " + i);
        }

        // Mass at t=0 should match launch mass
        assertEquals(launchMass, massTotal[0], 1e-6, "Mass at t=0 should equal launch mass");

        // Inertia (should be positive)
        assertTrue(getScalar(mat, "Ixx_launch") >= 0, "Ixx must be non-negative");
        assertTrue(getScalar(mat, "Iyy_launch") >= 0, "Iyy must be non-negative");
        assertTrue(getScalar(mat, "Izz_launch") >= 0, "Izz must be non-negative");

        // Reference geometry
        double refArea = getScalar(mat, "ref_area");
        double refLength = getScalar(mat, "ref_length");
        assertTrue(refArea > 0, "Reference area must be positive");
        assertTrue(refLength > 0, "Reference length must be positive");

        // CD table (now 3D: Mach x AoA x Alt)
        Matrix cdTable = mat.getMatrix("CD_table");
        assertNotNull(cdTable, "CD_table must exist");
        int[] cdDims = cdTable.getDimensions();
        assertTrue(cdDims.length == 3, "CD_table must be 3D (Mach x AoA x Alt), got " + cdDims.length + "D");
        int nMach = cdDims[0];
        int nAoA = cdDims[1];
        int nAlt = cdDims[2];
        assertTrue(nMach >= 2, "CD table must have multiple Mach points");
        assertTrue(nAoA >= 1, "CD table must have AoA points");
        assertTrue(nAlt >= 1, "CD table must have altitude points");

        // CD should be positive for all non-zero Mach (check at first alt)
        for (int i = 0; i < nMach; i++) {
            double cd = cdTable.getDouble(new int[]{i, 0, 0});
            assertTrue(cd > 0 && cd < 10, "CD should be positive and reasonable at Mach idx " + i + " (got " + cd + ")");
        }

        // Breakpoints
        double[] machBp = getRow(mat, "mach_bp");
        double[] aoaBp = getRow(mat, "aoa_bp");
        double[] altBp = getRow(mat, "alt_bp");
        assertEquals(nMach, machBp.length, "Mach breakpoints must match CD table dim 1");
        assertEquals(nAoA, aoaBp.length, "AoA breakpoints must match CD table dim 2");
        assertEquals(nAlt, altBp.length, "Alt breakpoints must match CD table dim 3");

        // Additional aero tables should exist
        assertNotNull(mat.getMatrix("CN_table"), "CN_table must exist");
        assertNotNull(mat.getMatrix("Cm_table"), "Cm_table must exist");
        assertNotNull(mat.getMatrix("Croll_table"), "Croll_table must exist");
        assertNotNull(mat.getMatrix("CDaxial_table"), "CDaxial_table must exist");

        mat.close();
    }

    @Test
    @Order(2)
    @DisplayName("Export with simulation validation data")
    void testWithSimulationValidation() throws Exception {
        String orkFile = EXAMPLES_DIR + "A simple model rocket.ork";
        String matPath = tempDir.resolve("simple_rocket_sim.mat").toString();

        int exitCode = runExporter(orkFile, matPath, "--run-sim");
        assertEquals(0, exitCode, "Exporter with --run-sim should succeed");

        Mat5File mat = loadMat(matPath);

        // OR simulation summary values should exist and be reasonable
        double maxAlt = getScalar(mat, "or_max_altitude");
        double maxVel = getScalar(mat, "or_max_velocity");
        double maxAccel = getScalar(mat, "or_max_acceleration");
        double flightTime = getScalar(mat, "or_flight_time");

        assertTrue(maxAlt > 10 && maxAlt < 5000, "Max altitude should be reasonable (got " + maxAlt + " m)");
        assertTrue(maxVel > 5 && maxVel < 500, "Max velocity should be reasonable (got " + maxVel + " m/s)");
        assertTrue(maxAccel > 0, "Max acceleration should be positive");
        assertTrue(flightTime > 1 && flightTime < 300, "Flight time should be reasonable (got " + flightTime + " s)");

        // Time-series data should exist
        Matrix orTime = mat.getMatrix("or_time");
        Matrix orAlt = mat.getMatrix("or_altitude");
        Matrix orVel = mat.getMatrix("or_velocity");

        assertNotNull(orTime, "or_time should exist");
        assertNotNull(orAlt, "or_altitude should exist");
        assertNotNull(orVel, "or_velocity should exist");

        int nPoints = orTime.getNumCols();
        assertTrue(nPoints > 50, "Should have many time series points (got " + nPoints + ")");
        assertEquals(nPoints, orAlt.getNumCols(), "Altitude array must match time array length");
        assertEquals(nPoints, orVel.getNumCols(), "Velocity array must match time array length");

        // Time should be monotonically increasing
        for (int i = 1; i < nPoints; i++) {
            assertTrue(orTime.getDouble(0, i) > orTime.getDouble(0, i - 1),
                    "Time should be monotonically increasing at index " + i);
        }

        mat.close();
    }

    @Test
    @Order(3)
    @DisplayName("Export high-power rocket — multi-stage handling")
    void testHighPowerRocket() throws Exception {
        String orkFile = EXAMPLES_DIR + "Two stage high power rocket.ork";
        String matPath = tempDir.resolve("two_stage.mat").toString();

        // Skip if the example doesn't exist (some builds may not have it)
        if (!new File(orkFile).exists()) {
            System.out.println("Skipping: " + orkFile + " not found");
            return;
        }

        int exitCode = runExporter(orkFile, matPath);
        assertEquals(0, exitCode, "Exporter should succeed for high-power rocket");

        Mat5File mat = loadMat(matPath);

        double launchMass = getScalar(mat, "mass_launch");
        assertTrue(launchMass > 0.5, "High-power rocket should have significant mass (got " + launchMass + " kg)");

        double burnTime = getScalar(mat, "burn_time");
        assertTrue(burnTime > 0, "Burn time should be positive");

        mat.close();
    }

    @Test
    @Order(4)
    @DisplayName("Export clustered motors — thrust summation")
    void testClusteredMotors() throws Exception {
        String orkFile = EXAMPLES_DIR + "Clustered motors.ork";
        String matPath = tempDir.resolve("clustered.mat").toString();

        if (!new File(orkFile).exists()) {
            System.out.println("Skipping: " + orkFile + " not found");
            return;
        }

        int exitCode = runExporter(orkFile, matPath);
        assertEquals(0, exitCode, "Exporter should succeed for clustered motors");

        Mat5File mat = loadMat(matPath);

        double nMotors = getScalar(mat, "n_motors");
        assertTrue(nMotors >= 1, "Should have at least 1 motor (got " + nMotors + ")");

        // If cluster, thrust should be scaled
        double[] thrustForce = getRow(mat, "thrust_force");
        double maxThrust = 0;
        for (double t : thrustForce) {
            maxThrust = Math.max(maxThrust, t);
        }
        assertTrue(maxThrust > 0, "Max thrust should be positive");

        mat.close();
    }

    @Test
    @Order(5)
    @DisplayName("Missing .ork file returns error exit code")
    void testMissingOrkFile() {
        String matPath = tempDir.resolve("nonexistent.mat").toString();
        int exitCode = runExporter("/tmp/nonexistent_rocket.ork", matPath);
        assertEquals(1, exitCode, "Should return error code for missing file");
        assertFalse(new File(matPath).exists(), "No MAT file should be created for missing input");
    }

    @Test
    @Order(6)
    @DisplayName("Simulation settings are correctly exported")
    void testSimulationSettings() throws Exception {
        String orkFile = EXAMPLES_DIR + "A simple model rocket.ork";
        String matPath = tempDir.resolve("simple_settings.mat").toString();

        int exitCode = runExporter(orkFile, matPath);
        assertEquals(0, exitCode);

        Mat5File mat = loadMat(matPath);

        // Launch rod settings should be present
        double rodLength = getScalar(mat, "launch_rod_length");
        assertTrue(rodLength > 0, "Launch rod length should be positive (got " + rodLength + ")");

        double rodAngle = getScalar(mat, "launch_rod_angle");
        assertTrue(rodAngle >= 0 && rodAngle < Math.PI, "Launch rod angle should be in [0, pi)");

        double timeStep = getScalar(mat, "time_step");
        assertTrue(timeStep > 0 && timeStep < 1, "Time step should be a small positive number");

        // Atmosphere
        double launchTemp = getScalar(mat, "launch_temperature");
        assertTrue(launchTemp > 200 && launchTemp < 350, "Temperature should be reasonable in Kelvin (got " + launchTemp + ")");

        double launchPressure = getScalar(mat, "launch_pressure");
        assertTrue(launchPressure > 80000 && launchPressure < 110000, "Pressure should be near sea level (got " + launchPressure + ")");

        mat.close();
    }

    @Test
    @Order(7)
    @DisplayName("Geometry data is reasonable")
    void testGeometryData() throws Exception {
        String orkFile = EXAMPLES_DIR + "A simple model rocket.ork";
        String matPath = tempDir.resolve("simple_geom.mat").toString();

        int exitCode = runExporter(orkFile, matPath);
        assertEquals(0, exitCode);

        Mat5File mat = loadMat(matPath);

        double rocketLength = getScalar(mat, "rocket_length");
        assertTrue(rocketLength > 0.05 && rocketLength < 5.0,
                "Rocket length should be reasonable (got " + rocketLength + " m)");

        double finCount = getScalar(mat, "fin_count");
        assertTrue(finCount >= 3 && finCount <= 8,
                "Fin count should be 3-8 for a typical rocket (got " + finCount + ")");

        double refArea = getScalar(mat, "ref_area");
        double refLength = getScalar(mat, "ref_length");
        // Check area ≈ π(d/2)² relationship
        double expectedArea = Math.PI * (refLength / 2) * (refLength / 2);
        assertEquals(expectedArea, refArea, 1e-6, "ref_area should equal π(ref_length/2)²");

        mat.close();
    }

    @Test
    @Order(8)
    @DisplayName("Time-varying inertia has correct shape and values")
    void testTimeVaryingInertia() throws Exception {
        String orkFile = EXAMPLES_DIR + "A simple model rocket.ork";
        String matPath = tempDir.resolve("simple_inertia.mat").toString();

        int exitCode = runExporter(orkFile, matPath);
        assertEquals(0, exitCode);

        Mat5File mat = loadMat(matPath);

        double[] ixxTime = getRow(mat, "Ixx_vs_time");
        double[] iyyTime = getRow(mat, "Iyy_vs_time");
        double[] izzTime = getRow(mat, "Izz_vs_time");
        double[] massTime = getRow(mat, "mass_time");
        double[] cgxTime = getRow(mat, "cg_x_vs_time");

        // All should have the same length
        assertEquals(massTime.length, ixxTime.length, "Ixx_vs_time must match mass_time length");
        assertEquals(massTime.length, iyyTime.length, "Iyy_vs_time must match mass_time length");
        assertEquals(massTime.length, izzTime.length, "Izz_vs_time must match mass_time length");
        assertEquals(massTime.length, cgxTime.length, "cg_x_vs_time must match mass_time length");

        // All inertia values should be non-negative
        for (int i = 0; i < ixxTime.length; i++) {
            assertTrue(ixxTime[i] >= 0, "Ixx must be non-negative at index " + i);
            assertTrue(iyyTime[i] >= 0, "Iyy must be non-negative at index " + i);
            assertTrue(izzTime[i] >= 0, "Izz must be non-negative at index " + i);
        }

        // CG should move (at least slightly) as propellant burns
        if (cgxTime.length > 2) {
            // CG position should be a valid number
            for (double cg : cgxTime) {
                assertFalse(Double.isNaN(cg), "CG position must not be NaN");
            }
        }

        mat.close();
    }

    @Test
    @Order(9)
    @DisplayName("CD table breakdown components sum correctly")
    void testCdBreakdown() throws Exception {
        String orkFile = EXAMPLES_DIR + "A simple model rocket.ork";
        String matPath = tempDir.resolve("simple_cd_breakdown.mat").toString();

        int exitCode = runExporter(orkFile, matPath);
        assertEquals(0, exitCode);

        Mat5File mat = loadMat(matPath);

        Matrix cdTable = mat.getMatrix("CD_table");
        Matrix pressureCd = mat.getMatrix("pressureCD_table");
        Matrix frictionCd = mat.getMatrix("frictionCD_table");
        Matrix baseCd = mat.getMatrix("baseCD_table");

        assertNotNull(cdTable);
        assertNotNull(pressureCd);
        assertNotNull(frictionCd);
        assertNotNull(baseCd);

        // Total CD should be approximately >= sum of components (there may be rounding/override)
        int[] cdDims = cdTable.getDimensions();
        int nMach = cdDims[0];
        int nAoA = cdDims[1];

        for (int i = 0; i < Math.min(nMach, 3); i++) {
            for (int j = 0; j < Math.min(nAoA, 2); j++) {
                double total = cdTable.getDouble(new int[]{i, j, 0});
                double pressure = pressureCd.getDouble(new int[]{i, j, 0});
                double friction = frictionCd.getDouble(new int[]{i, j, 0});
                double base = baseCd.getDouble(new int[]{i, j, 0});

                // All components should be non-negative
                assertTrue(pressure >= 0, "Pressure CD must be non-negative");
                assertTrue(friction >= 0, "Friction CD must be non-negative");
                assertTrue(base >= 0, "Base CD must be non-negative");

                // Total should be >= sum of named components (it may include override)
                double componentSum = pressure + friction + base;
                assertTrue(total >= componentSum * 0.8,
                        "Total CD (" + total + ") should be >= ~sum of components (" + componentSum + ")");
            }
        }

        mat.close();
    }

    // ─── New v2.0 Tests ──────────────────────────────────────────────

    @Test
    @Order(10)
    @DisplayName("Metadata struct exists and has required fields")
    void testMetadataStruct() throws Exception {
        String orkFile = EXAMPLES_DIR + "A simple model rocket.ork";
        String matPath = tempDir.resolve("simple_meta.mat").toString();

        int exitCode = runExporter(orkFile, matPath);
        assertEquals(0, exitCode);

        Mat5File mat = loadMat(matPath);

        // meta struct must exist
        Struct meta = mat.getStruct("meta");
        assertNotNull(meta, "meta struct must exist in MAT file");

        // Check key string fields are non-empty
        Char orVersion = meta.getChar("openrocket_version");
        assertNotNull(orVersion, "meta.openrocket_version must exist");

        Char exporterVersion = meta.getChar("exporter_version");
        assertNotNull(exporterVersion, "meta.exporter_version must exist");
        assertEquals("2.1.0", exporterVersion.getString(), "Exporter version must be 2.1.0");

        Char sha256 = meta.getChar("ork_sha256");
        assertNotNull(sha256, "meta.ork_sha256 must exist");
        assertTrue(sha256.getString().length() == 64, "SHA-256 hash must be 64 hex chars");

        // Check coordinate conventions
        Char coordOrigin = meta.getChar("coord_origin");
        assertNotNull(coordOrigin, "meta.coord_origin must exist");
        assertEquals("nose_tip", coordOrigin.getString());

        Char inertiaFrame = meta.getChar("inertia_frame");
        assertNotNull(inertiaFrame, "meta.inertia_frame must exist");

        mat.close();
    }

    @Test
    @Order(11)
    @DisplayName("Separated mass properties are consistent")
    void testSeparatedMassProperties() throws Exception {
        String orkFile = EXAMPLES_DIR + "A simple model rocket.ork";
        String matPath = tempDir.resolve("simple_separated_mass.mat").toString();

        int exitCode = runExporter(orkFile, matPath);
        assertEquals(0, exitCode);

        Mat5File mat = loadMat(matPath);

        double mStructure = getScalar(mat, "m_structure");
        double mMotorLaunch = getScalar(mat, "m_motor_launch");
        double mTotalLaunch = getScalar(mat, "m_total_launch");
        double mTotalBurnout = getScalar(mat, "m_total_burnout");

        assertTrue(mStructure > 0, "m_structure must be positive");
        assertTrue(mMotorLaunch > 0, "m_motor_launch must be positive");
        assertTrue(mTotalLaunch > 0, "m_total_launch must be positive");
        assertTrue(mTotalBurnout > 0, "m_total_burnout must be positive");

        // m_total_launch ≈ m_structure + m_motor_launch
        assertEquals(mTotalLaunch, mStructure + mMotorLaunch, 1e-4,
                "m_total_launch must ≈ m_structure + m_motor_launch");

        // Structure inertias must exist and be non-negative
        assertTrue(getScalar(mat, "I_structure_yy") >= 0, "I_structure_yy must be non-negative");
        assertTrue(getScalar(mat, "I_motor_launch_yy") >= 0, "I_motor_launch_yy must be non-negative");
        assertTrue(getScalar(mat, "I_total_launch_yy") >= 0, "I_total_launch_yy must be non-negative");

        mat.close();
    }

    @Test
    @Order(12)
    @DisplayName("Time arrays are strictly monotonic")
    void testMonotonicTimeGuarantee() throws Exception {
        String orkFile = EXAMPLES_DIR + "A simple model rocket.ork";
        String matPath = tempDir.resolve("simple_monotonic.mat").toString();

        int exitCode = runExporter(orkFile, matPath);
        assertEquals(0, exitCode);

        Mat5File mat = loadMat(matPath);

        // Check thrust_time is strictly increasing
        double[] thrustTime = getRow(mat, "thrust_time");
        for (int i = 1; i < thrustTime.length; i++) {
            assertTrue(thrustTime[i] > thrustTime[i - 1],
                    "thrust_time must be strictly increasing at index " + i +
                    " (got " + thrustTime[i-1] + " -> " + thrustTime[i] + ")");
        }

        // Check mass_time is strictly increasing
        double[] massTime = getRow(mat, "mass_time");
        for (int i = 1; i < massTime.length; i++) {
            assertTrue(massTime[i] > massTime[i - 1],
                    "mass_time must be strictly increasing at index " + i +
                    " (got " + massTime[i-1] + " -> " + massTime[i] + ")");
        }

        mat.close();
    }

    @Test
    @Order(13)
    @DisplayName("dI/dt arrays exist and have correct length")
    void testDIdtArrays() throws Exception {
        String orkFile = EXAMPLES_DIR + "A simple model rocket.ork";
        String matPath = tempDir.resolve("simple_didt.mat").toString();

        int exitCode = runExporter(orkFile, matPath);
        assertEquals(0, exitCode);

        Mat5File mat = loadMat(matPath);

        double[] massTime = getRow(mat, "mass_time");
        double[] dIxxDt = getRow(mat, "dIxx_dt");
        double[] dIyyDt = getRow(mat, "dIyy_dt");
        double[] dIzzDt = getRow(mat, "dIzz_dt");

        assertEquals(massTime.length, dIxxDt.length, "dIxx_dt must have same length as mass_time");
        assertEquals(massTime.length, dIyyDt.length, "dIyy_dt must have same length as mass_time");
        assertEquals(massTime.length, dIzzDt.length, "dIzz_dt must have same length as mass_time");

        // dI/dt values should be finite
        for (int i = 0; i < dIxxDt.length; i++) {
            assertFalse(Double.isNaN(dIxxDt[i]), "dIxx_dt must not be NaN at index " + i);
            assertFalse(Double.isInfinite(dIxxDt[i]), "dIxx_dt must not be Inf at index " + i);
        }

        mat.close();
    }

    @Test
    @Order(14)
    @DisplayName("Fin cant angle is exported for all fin sets")
    void testFinCantAngle() throws Exception {
        String orkFile = EXAMPLES_DIR + "A simple model rocket.ork";
        String matPath = tempDir.resolve("simple_fins.mat").toString();

        int exitCode = runExporter(orkFile, matPath);
        assertEquals(0, exitCode);

        Mat5File mat = loadMat(matPath);

        double nFinsets = getScalar(mat, "n_finsets");
        assertTrue(nFinsets >= 1, "Should have at least 1 fin set");

        // fin0_cant_angle_rad must exist
        Matrix cantAngle = mat.getMatrix("fin0_cant_angle_rad");
        assertNotNull(cantAngle, "fin0_cant_angle_rad must exist");
        double cantVal = cantAngle.getDouble(0, 0);
        assertFalse(Double.isNaN(cantVal), "fin0_cant_angle_rad must not be NaN");
        // For a simple rocket, cant angle is typically 0
        assertEquals(0.0, cantVal, 1e-6, "Simple rocket fin cant angle should be 0");

        // Also verify per-finset count, span, thickness
        Matrix fin0Count = mat.getMatrix("fin0_count");
        assertNotNull(fin0Count, "fin0_count must exist");
        assertTrue(fin0Count.getDouble(0, 0) >= 3, "Fin count should be >= 3");

        mat.close();
    }

    @Test
    @Order(15)
    @DisplayName("Burnout tail extends past burn time with zero thrust")
    void testBurnoutTail() throws Exception {
        String orkFile = EXAMPLES_DIR + "A simple model rocket.ork";
        String matPath = tempDir.resolve("simple_burnout.mat").toString();

        int exitCode = runExporter(orkFile, matPath);
        assertEquals(0, exitCode);

        Mat5File mat = loadMat(matPath);

        double[] thrustTime = getRow(mat, "thrust_time");
        double[] thrustForce = getRow(mat, "thrust_force");
        double burnTime = getScalar(mat, "burn_time");

        // Last thrust value must be 0.0
        assertEquals(0.0, thrustForce[thrustForce.length - 1], 1e-9,
                "Last thrust value must be 0.0 (far-future hold)");

        // Last time point should extend well past burn time (by ~100s)
        assertTrue(thrustTime[thrustTime.length - 1] > burnTime + 50,
                "thrust_time should extend well past burn_time (got " +
                thrustTime[thrustTime.length - 1] + " vs burn_time=" + burnTime + ")");

        // Second-to-last should also be 0
        assertEquals(0.0, thrustForce[thrustForce.length - 2], 1e-9,
                "Second-to-last thrust must also be 0.0 (burnout zero crossing)");

        mat.close();
    }

    @Test
    @Order(16)
    @DisplayName("Additional sim channels are exported with --run-sim")
    void testAdditionalSimChannels() throws Exception {
        String orkFile = EXAMPLES_DIR + "A simple model rocket.ork";
        String matPath = tempDir.resolve("simple_channels.mat").toString();

        int exitCode = runExporter(orkFile, matPath, "--run-sim");
        assertEquals(0, exitCode);

        Mat5File mat = loadMat(matPath);

        // New channels should exist
        Matrix orAoa = mat.getMatrix("or_aoa");
        assertNotNull(orAoa, "or_aoa must exist when --run-sim is used");
        assertTrue(orAoa.getNumCols() > 0, "or_aoa should have data points");

        Matrix orRollRate = mat.getMatrix("or_roll_rate");
        assertNotNull(orRollRate, "or_roll_rate must exist when --run-sim is used");

        Matrix orCg = mat.getMatrix("or_cg");
        assertNotNull(orCg, "or_cg must exist when --run-sim is used");

        Matrix orCp = mat.getMatrix("or_cp");
        assertNotNull(orCp, "or_cp must exist when --run-sim is used");

        Matrix orStability = mat.getMatrix("or_stability");
        assertNotNull(orStability, "or_stability must exist when --run-sim is used");

        mat.close();
    }

    // ─── New v2.1 Tests ──────────────────────────────────────────────

    @Test
    @Order(17)
    @DisplayName("3D aero tables have correct Mach x AoA x Alt dimensions")
    void test3DAeroTableDimensions() throws Exception {
        String orkFile = EXAMPLES_DIR + "A simple model rocket.ork";
        String matPath = tempDir.resolve("aero3d.mat").toString();

        int exitCode = runExporter(orkFile, matPath, "--alt-max", "5000");
        assertEquals(0, exitCode, "Exporter should succeed");

        Mat5File mat = loadMat(matPath);

        // CD_table should be 3D
        Matrix cdTable = mat.getMatrix("CD_table");
        assertNotNull(cdTable, "CD_table must exist");
        int[] dims = cdTable.getDimensions();
        assertEquals(3, dims.length, "CD_table must be 3-dimensional");

        // alt_bp should exist and match dimension 3
        double[] altBp = getRow(mat, "alt_bp");
        assertEquals(dims[2], altBp.length, "alt_bp length must match CD_table dim 3");
        assertEquals(0.0, altBp[0], 1e-6, "First altitude breakpoint should be 0");
        assertEquals(5000.0, altBp[altBp.length - 1], 1e-6, "Last altitude breakpoint should be 5000");

        // CD at higher altitude should differ from sea level (Reynolds effect)
        double cdSeaLevel = cdTable.getDouble(new int[]{1, 0, 0});
        double cdHighAlt = cdTable.getDouble(new int[]{1, 0, dims[2] - 1});
        assertNotEquals(cdSeaLevel, cdHighAlt, 1e-9,
                "CD should differ between sea level and " + altBp[altBp.length - 1] + "m");

        // All aero tables should be 3D
        for (String tableName : new String[]{"CN_table", "Cm_table", "Croll_table",
                "CrollDamp_table", "CrollForce_table"}) {
            Matrix table = mat.getMatrix(tableName);
            assertNotNull(table, tableName + " must exist");
            int[] tDims = table.getDimensions();
            assertEquals(3, tDims.length, tableName + " must be 3D");
            assertEquals(dims[0], tDims[0], tableName + " dim1 must match CD_table");
            assertEquals(dims[1], tDims[1], tableName + " dim2 must match CD_table");
            assertEquals(dims[2], tDims[2], tableName + " dim3 must match CD_table");
        }

        mat.close();
    }

    @Test
    @Order(18)
    @DisplayName("Motor identity metadata is exported")
    void testMotorIdentityMetadata() throws Exception {
        String orkFile = EXAMPLES_DIR + "A simple model rocket.ork";
        String matPath = tempDir.resolve("motor_identity.mat").toString();

        int exitCode = runExporter(orkFile, matPath);
        assertEquals(0, exitCode, "Exporter should succeed");

        Mat5File mat = loadMat(matPath);

        // Motor identity fields must exist
        Char manufacturer = mat.getChar("motor_manufacturer");
        assertNotNull(manufacturer, "motor_manufacturer must exist");
        assertTrue(manufacturer.getString().length() > 0, "motor_manufacturer must be non-empty");

        double totalImpulse = getScalar(mat, "motor_total_impulse");
        assertTrue(totalImpulse > 0, "motor_total_impulse must be positive (got " + totalImpulse + ")");

        double avgThrust = getScalar(mat, "motor_avg_thrust");
        assertTrue(avgThrust > 0, "motor_avg_thrust must be positive");

        double maxThrust = getScalar(mat, "motor_max_thrust");
        assertTrue(maxThrust >= avgThrust, "motor_max_thrust must be >= avg_thrust");

        Char impulseClass = mat.getChar("motor_impulse_class");
        assertNotNull(impulseClass, "motor_impulse_class must exist");

        // Motor identity in meta struct too
        Struct meta = mat.getStruct("meta");
        assertNotNull(meta.getChar("motor_manufacturer"), "meta.motor_manufacturer must exist");
        assertNotNull(meta.getChar("motor_type"), "meta.motor_type must exist");

        mat.close();
    }

    @Test
    @Order(19)
    @DisplayName("Scenario and wind fields are exported")
    void testScenarioFields() throws Exception {
        String orkFile = EXAMPLES_DIR + "A simple model rocket.ork";
        String matPath = tempDir.resolve("scenario.mat").toString();

        int exitCode = runExporter(orkFile, matPath);
        assertEquals(0, exitCode, "Exporter should succeed");

        Mat5File mat = loadMat(matPath);

        // Wind fields
        double windSpeed = getScalar(mat, "wind_speed_avg");
        assertTrue(windSpeed >= 0, "wind_speed_avg must be non-negative");

        double windDir = getScalar(mat, "wind_direction");
        assertFalse(Double.isNaN(windDir), "wind_direction must not be NaN");

        double turbulence = getScalar(mat, "wind_turbulence_intensity");
        assertTrue(turbulence >= 0 && turbulence <= 1.0,
                "wind_turbulence_intensity should be in [0,1] (got " + turbulence + ")");

        // ISA and humidity
        double isa = getScalar(mat, "isa_atmosphere");
        assertTrue(isa == 0.0 || isa == 1.0, "isa_atmosphere must be 0 or 1");

        double humidity = getScalar(mat, "launch_relative_humidity");
        assertTrue(humidity >= 0 && humidity <= 1.0,
                "launch_relative_humidity should be in [0,1] (got " + humidity + ")");

        // Scenario-prefixed aliases
        assertEquals(windSpeed, getScalar(mat, "scenario_wind_speed"), 1e-9,
                "scenario_wind_speed must equal wind_speed_avg");
        assertEquals(getScalar(mat, "launch_altitude"), getScalar(mat, "scenario_launch_altitude"), 1e-9,
                "scenario_launch_altitude must equal launch_altitude");

        mat.close();
    }

    @Test
    @Order(20)
    @DisplayName("3x3 inertia tensor has correct shape")
    void testInertiaTensor() throws Exception {
        String orkFile = EXAMPLES_DIR + "A simple model rocket.ork";
        String matPath = tempDir.resolve("tensor.mat").toString();

        int exitCode = runExporter(orkFile, matPath);
        assertEquals(0, exitCode, "Exporter should succeed");

        Mat5File mat = loadMat(matPath);

        Matrix tensor = mat.getMatrix("I_tensor_vs_time");
        assertNotNull(tensor, "I_tensor_vs_time must exist");

        int[] dims = tensor.getDimensions();
        assertEquals(3, dims.length, "I_tensor_vs_time must be 3D");
        assertEquals(3, dims[0], "I_tensor dim 1 must be 3");
        assertEquals(3, dims[1], "I_tensor dim 2 must be 3");
        assertTrue(dims[2] >= 2, "I_tensor must have multiple time steps");

        // Check diagonal is positive at t=0
        double ixx0 = tensor.getDouble(new int[]{0, 0, 0});
        double iyy0 = tensor.getDouble(new int[]{1, 1, 0});
        double izz0 = tensor.getDouble(new int[]{2, 2, 0});
        assertTrue(ixx0 >= 0, "I_tensor(1,1,1) = Ixx must be non-negative");
        assertTrue(iyy0 >= 0, "I_tensor(2,2,1) = Iyy must be non-negative");
        assertTrue(izz0 >= 0, "I_tensor(3,3,1) = Izz must be non-negative");

        // Off-diagonals should be zero
        assertEquals(0.0, tensor.getDouble(new int[]{0, 1, 0}), 1e-15, "I_tensor(1,2) must be 0");
        assertEquals(0.0, tensor.getDouble(new int[]{1, 0, 0}), 1e-15, "I_tensor(2,1) must be 0");
        assertEquals(0.0, tensor.getDouble(new int[]{0, 2, 0}), 1e-15, "I_tensor(1,3) must be 0");
        assertEquals(0.0, tensor.getDouble(new int[]{2, 0, 0}), 1e-15, "I_tensor(3,1) must be 0");

        // Diagonal should match separate Ixx/Iyy/Izz arrays
        double[] ixxArr = getRow(mat, "Ixx_vs_time");
        assertEquals(ixxArr[0], ixx0, 1e-12,
                "I_tensor(1,1,1) must match Ixx_vs_time(1)");

        mat.close();
    }

    @Test
    @Order(21)
    @DisplayName("Invalid sim index lists available simulations")
    void testSimListOnInvalidIndex() throws Exception {
        String orkFile = EXAMPLES_DIR + "A simple model rocket.ork";
        String matPath = tempDir.resolve("invalid_sim.mat").toString();

        // Use an out-of-range sim index
        int exitCode = runExporter(orkFile, matPath, "--sim", "999");
        assertEquals(1, exitCode, "Should fail with invalid sim index");
        assertFalse(new File(matPath).exists(), "No MAT file should be created");
    }

    @Test
    @Order(22)
    @DisplayName("Rail exit velocity is exported with --run-sim")
    void testRailExitData() throws Exception {
        String orkFile = EXAMPLES_DIR + "A simple model rocket.ork";
        String matPath = tempDir.resolve("rail_exit.mat").toString();

        int exitCode = runExporter(orkFile, matPath, "--run-sim");
        assertEquals(0, exitCode, "Exporter with --run-sim should succeed");

        Mat5File mat = loadMat(matPath);

        double railExitVel = getScalar(mat, "or_rail_exit_velocity");
        assertTrue(railExitVel > 0 && railExitVel < 200,
                "Rail exit velocity should be reasonable (got " + railExitVel + " m/s)");

        mat.close();
    }
}
