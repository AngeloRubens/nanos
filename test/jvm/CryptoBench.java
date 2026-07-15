import com.sun.management.HotSpotDiagnosticMXBean;
import java.lang.management.ManagementFactory;
import java.security.MessageDigest;
import java.util.Locale;
import java.util.Random;
import java.util.zip.CRC32C;
import javax.crypto.Cipher;
import javax.crypto.spec.GCMParameterSpec;
import javax.crypto.spec.SecretKeySpec;

/**
 * A/B benchmark for the nanos arm64 HWCAP patch (#20).
 *
 * Same class runs on an UNPATCHED image (crypto intrinsics forced OFF by
 * HotSpot -> software fallback) and a PATCHED image (HW AES/SHA/CRC32 ON).
 * Pure JDK, no dependencies -> bakes straight into a nanos image.
 *
 * It prints, in order:
 *   1) the crypto flags actually in effect  -> self-documents which mode it ran
 *   2) deterministic reference outputs (KAT) -> MUST be identical HW vs SW
 *                                               (proves speed != skipped work)
 *   3) steady-state throughput (MB/s)        -> the number you compare
 *
 * All numeric output is Locale.ROOT (plain "1371.1") so CI can parse it.
 *
 * Tunables (system properties):
 *   -Dbench.size=<bytes>  (default 1 MiB)  -Dbench.warmup=<iters> (default 30000)
 *   -Dbench.ms=<millis>   (default 3000)   -Dbench.rounds=<n>     (default 3)
 */
public class CryptoBench {

    static final int  DATA_SIZE  = Integer.getInteger("bench.size", 1 << 20);
    static final int  WARMUP     = Integer.getInteger("bench.warmup", 30_000);
    static final long MEASURE_MS = Long.getLong("bench.ms", 3000L);
    static final int  ROUNDS     = Integer.getInteger("bench.rounds", 3);

    // blackhole: consumed at the end so the JIT can't elide the work
    static long sink;

    public static void main(String[] args) throws Exception {
        System.out.println("=== CryptoBench  (nanos HWCAP #20 A/B) ===");
        System.out.printf(Locale.ROOT, "jvm=%s  arch=%s%n",
                System.getProperty("java.vm.version"), System.getProperty("os.arch"));
        System.out.printf(Locale.ROOT, "data=%d KiB  warmup=%d  measure=%dms  rounds=%d%n%n",
                DATA_SIZE >> 10, WARMUP, MEASURE_MS, ROUNDS);

        System.out.println("-- crypto flags in effect --");
        for (String f : new String[]{
                "UseAES", "UseAESIntrinsics", "UseGHASHIntrinsics",
                "UseSHA", "UseSHA256Intrinsics", "UseSHA512Intrinsics",
                "UseCRC32", "UseCRC32CIntrinsics", "UseChaCha20Intrinsics"}) {
            printFlag(f);
        }
        System.out.println();

        // deterministic data -> reference outputs are reproducible across runs
        byte[] data = new byte[DATA_SIZE];
        new Random(0xC0FFEEL).nextBytes(data);

        System.out.println("-- correctness (must match HW vs SW) --");
        correctness(data);
        System.out.println();

        System.out.println("-- throughput --");
        benchAesGcm(data);
        benchSha256(data);
        benchCrc32c(data);

        System.out.println("(sink=" + sink + ")");
    }

    // ------------------------------------------------------------------ flags
    static void printFlag(String name) {
        try {
            HotSpotDiagnosticMXBean b =
                ManagementFactory.getPlatformMXBean(HotSpotDiagnosticMXBean.class);
            System.out.printf(Locale.ROOT, "  %-22s = %s%n", name, b.getVMOption(name).getValue());
        } catch (Exception e) {
            System.out.printf(Locale.ROOT, "  %-22s = <n/a>%n", name);
        }
    }

    // ------------------------------------------------------- correctness (KAT)
    // Fixed key/IV/data -> deterministic. If the HW path corrupted state or the
    // intrinsic were wrong, these hex lines would differ between the two runs.
    static void correctness(byte[] data) throws Exception {
        byte[] key = new byte[32];
        byte[] iv  = new byte[12];
        for (int i = 0; i < key.length; i++) key[i] = (byte) (i * 7 + 1);
        for (int i = 0; i < iv.length;  i++) iv[i]  = (byte) (i * 3 + 5);

        Cipher c = Cipher.getInstance("AES/GCM/NoPadding");
        c.init(Cipher.ENCRYPT_MODE, new SecretKeySpec(key, "AES"),
               new GCMParameterSpec(128, iv));
        byte[] ct = c.doFinal(data);
        byte[] gcmTag = new byte[16];
        System.arraycopy(ct, ct.length - 16, gcmTag, 0, 16);

        byte[] sha = MessageDigest.getInstance("SHA-256").digest(data);

        CRC32C crc = new CRC32C();
        crc.update(data);

        System.out.println("  AES-GCM tag  = " + hex(gcmTag));
        System.out.println("  SHA-256      = " + hex(sha));
        System.out.printf(Locale.ROOT, "  CRC32C       = %08x%n", crc.getValue());
    }

    // -------------------------------------------------- AES-256-GCM (AES + GHASH)
    static void benchAesGcm(byte[] data) throws Exception {
        byte[] key = new byte[32];
        new Random(1).nextBytes(key);
        SecretKeySpec sk = new SecretKeySpec(key, "AES");
        Cipher c = Cipher.getInstance("AES/GCM/NoPadding");
        byte[] iv = new byte[12];
        long[] ctr = {0};

        Runnable op = () -> {
            try {
                // GCM forbids (key,IV) reuse -> bump the IV every op
                long v = ++ctr[0];
                for (int i = 0; i < 8; i++) iv[i] = (byte) (v >>> (i * 8));
                c.init(Cipher.ENCRYPT_MODE, sk, new GCMParameterSpec(128, iv));
                sink += c.doFinal(data)[0];
            } catch (Exception e) { throw new RuntimeException(e); }
        };
        for (int i = 0; i < WARMUP; i++) op.run();
        report("AES-256-GCM", data.length, op);
    }

    // ---------------------------------------------------------------- SHA-256
    static void benchSha256(byte[] data) throws Exception {
        MessageDigest md = MessageDigest.getInstance("SHA-256");
        Runnable op = () -> sink += md.digest(data)[0]; // digest() updates+resets
        for (int i = 0; i < WARMUP; i++) op.run();
        report("SHA-256", data.length, op);
    }

    // ----------------------------------------------------------------- CRC32C
    static void benchCrc32c(byte[] data) {
        CRC32C crc = new CRC32C();
        Runnable op = () -> { crc.reset(); crc.update(data); sink += crc.getValue(); };
        for (int i = 0; i < WARMUP; i++) op.run();
        report("CRC32C", data.length, op);
    }

    // ----------------------------------------------- timed steady-state (MB/s)
    static void report(String name, int bytesPerOp, Runnable op) {
        double best = 0;
        for (int r = 0; r < ROUNDS; r++) {
            long t0 = System.nanoTime();
            long deadline = t0 + MEASURE_MS * 1_000_000L;
            long ops = 0;
            while (System.nanoTime() < deadline) { op.run(); ops++; }
            double secs = (System.nanoTime() - t0) / 1e9;
            double mbps = ops * (double) bytesPerOp / (1024 * 1024) / secs;
            best = Math.max(best, mbps);
            System.out.printf(Locale.ROOT, "  %-12s round %d: %10.1f MB/s  (%d ops)%n",
                    name, r, mbps, ops);
        }
        System.out.printf(Locale.ROOT, "  %-12s BEST  : %10.1f MB/s%n%n", name, best);
    }

    static String hex(byte[] b) {
        StringBuilder s = new StringBuilder(b.length * 2);
        for (byte x : b) s.append(String.format(Locale.ROOT, "%02x", x));
        return s.toString();
    }
}
