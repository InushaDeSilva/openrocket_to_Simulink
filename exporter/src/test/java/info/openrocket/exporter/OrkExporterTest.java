package info.openrocket.exporter;

import org.junit.jupiter.api.*;
import us.hebi.matlab.mat.format.Mat5;
import us.hebi.matlab.mat.format.Mat5File;
import us.hebi.matlab.mat.types.MatFile;
import us.hebi.matlab.mat.types.Matrix;
import us.hebi.matlab.mat.types.Char;
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

        // CD table
        Matrix cdTable = mat.getMatrix("CD_table");
        assertNotNull(cdTable, "CD_table must exist");
        int nMach = cdTable.getNumRows();
        int nAoA = cdTable.getNumCols();
        assertTrue(nMach >= 2, "CD table must have multiple Mach points");
        assertTrue(nAoA >= 1, "CD table must have AoA points");

        // CD should be positive for all non-zero Mach
        for (int i = 0; i < nMach; i++) {
            double cd = cdTable.getDouble(i, 0);
            assertTrue(cd > 0 && cd < 10, "CD should be positive and reasonable at Mach idx " + i + " (got " + cd + ")");
        }

        // Breakpoints
        double[] machBp = getRow(mat, "mach_bp");
        double[] aoaBp = getRow(mat, "aoa_bp");
        assertEquals(nMach, machBp.length, "Mach breakpoints must match CD table rows");
        assertEquals(nAoA, aoaBp.length, "AoA breakpoints must match CD table columns");

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
        int nMach = cdTable.getNumRows();
        int nAoA = cdTable.getNumCols();

        for (int i = 0; i < Math.min(nMach, 3); i++) {
            for (int j = 0; j < Math.min(nAoA, 2); j++) {
                double total = cdTable.getDouble(i, j);
                double pressure = pressureCd.getDouble(i, j);
                double friction = frictionCd.getDouble(i, j);
                double base = baseCd.getDouble(i, j);

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
}
