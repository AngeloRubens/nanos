# Stato del lavoro uncommit/ZGC — consegna per una sessione nuova

Aggiornato 2026-08-17. Ramo di lavoro: `ci/zgc-probe`. Base: `upstream/master` = `70b0ac3c`.
Non git-tracked di proposito? No: committarlo pure, serve a chi riprende.

## Le nove patch, tutte committate su `ci/zgc-probe`

| # | commit / oggetto | prova che la sostiene |
|---|---|---|
| 1 | vmap lock ↔ flush (`spin_lock_irq_flushing` su `p->vmap_lock`) | `vmap_flush_race` **rosso su master, verde da noi**; backtrace gdb a 4 CPU su build stock |
| 2 | `7ab90b71` page table lock ↔ flush (`pt_lock`) | stesso test: **28 round** senza, **64** con |
| 3 | `c8a98bac` traverse oltre le pagine che non ci sono | fault a `0x3b` = `INVALID_ADDRESS + offsetof(refcount)` = 0x3c |
| 4 | ordine lock dentro il page-table walk | riproduzione; backtrace su master con CPU ferma in `pagecache_scan_shared_mappings` |
| 5 | `656ddd23` lock del nodo perso in `pagecache_release_page` | trovato leggendo (un `return` dentro il lock) |
| 6 | `552f5567` pagina già libera → `deallocate(INVALID_ADDRESS)` | aritmetica `0xff..ff + 0x18 = 0x17` + disassemblato + 12 occorrenze CI, mai in `before` |
| 7 | completion del sync su contesto proprio | riproduzione |
| 8 | nodo rilasciato prima di chiamare il filesystem | riproduzione |
| 9 | `fs:` la punch che restituisce le pagine | `tmpfs_punch` **rosso su master** (`holed < full`); 10,3 GB restituiti in 30 min di ZGC |

Rami PR pronti su `70b0ac3c` (uno per patch) più `pr/tmpfs-uncommit`. Testi in `/tmp/soaklab/PR-testi.md`.

## I test, e cosa dimostra ciascuno

Tutti si eseguono con `make VCPUS=2 -j4 TARGET=<nome> run-noaccel`.
**`VCPUS=2` è obbligatorio**: senza, qemu gira con UNA cpu e le corse non esistono. È anche
la configurazione della CI dei manutentori (CircleCI, docker, `make VCPUS=2 test-noaccel`, niente KVM).

| test | dimostra | master | nostro |
|---|---|---|---|
| `vmap_flush_race` | #1 #2 #3 | round 0, piantato | 64, passa |
| `vmap_flush_jvmpath` | **la serie intera** | hang round 0, 5/5 | 64 round, 13/14 |
| `tmpfs_punch` | #9 | `holed < full` | passa |
| `tmpfs_punch_race` | — (imita ZGC) | — | 32, passa |
| `tmpfs_punch_nosync` | — (più vicino a ZGC: niente fsync) | — | 32, passa |
| `release_page_race` | mira a #5 | — | 40, passa |
| `tmpfs_commit_race` | nulla: passa su entrambi | passa | passa |

`tlbshootdown` (loro) passa su entrambi con VCPUS=2: smappa da un thread solo, non chiude il ciclo.

## Cose che ho affermato e poi smentito con i fatti — non ripeterle

- **"C'è un decimo difetto irrisolto (`mutex.c:96`)"**: FALSO sul nostro albero. Visto UNA volta;
  un agente non l'ha riprodotto in 48 boot (6 sotto KVM da 1200 round). Le patch #7 e #8 chiudono
  quel percorso. Gli assert raccolti vengono da prima di quelle due patch o dal nightly upstream.
- **"`tmpfs_punch` copre la #6"**: FALSO, ablazione fatta: passa anche senza `552f5567`.
- **"`tmpfs_commit_race` dimostra #7 e #8"**: FALSO, passa su master.
- **Attribuzione "a chi serve"**: sbagliata due volte a memoria (#3 e #4). Va verificata sul codice.

## Il decimo difetto, diagnosticato con gdb il 18/08 — leggere prima di toccare i lock

Catturato con lo stub gdb su 4 processori (`tmpfs_punch_nosync`, VCPUS=4, si pianta da noi e
passa su master):

    CPU0  tiene il node lock, PARCHEGGIATA in flush_gen_rlocked (flush.c:77)
          <- _flush_handler <- spin_lock_irq_flushing(&pt_lock) <- traverse_ptes
          <- pagecache_node_unmap_pages_sync   (dentro pagecache_lock_node)
    CPU1  page_invalidate_sync(rendezvous=1), aspetta joined == 4
          <- pagecache_node_free_pages <- fallocate     ← la NOSTRA punch
    CPU3  aspetta il node lock che CPU0 tiene -> non joina mai

**Il meccanismo**: `flush_gen_rlocked` non fa un ack, fa un *join e parcheggia*
(`fetch_and_add(&f->joined,1); while (f->wait) kern_pause();`, flush.c:75-78) perche'
l'iniziatore esegue `apply(completion)` con tutti gli altri congelati (flush.c:221-222).
Quel parcheggio e' corretto solo dall'handler dell'IPI, dove non si tiene nulla. Le nostre
#1 e #2 lo invocano da `spin_lock_irq_flushing`, cioe' **da dentro una sezione critica**.

**Non esiste una toppa locale.** Se il servizio non joina, l'iniziatore attende per sempre;
se joina, parcheggia tenendo lock. Le due strade vere, con precedenti letti sui sorgenti:

1. **aarch64: togliere IPI e rendezvous.** `invalidate()`/`flush_tlb()` (src/aarch64/page.c:18-31)
   sono gia' broadcast inner-shareable: il coordinamento software e' ridondante. Precedenti:
   OSv `arch/aarch64/mmu.cc:118-120`, Unikraft `plat/native/arch/arm64/.../tlb.h:45-52`,
   Linux `arch/arm64/include/asm/tlbflush.h:376-386`.
2. **Modello ad ack invece del rendezvous**: il bersaglio invalida, conferma e torna; solo
   l'iniziatore aspetta un contatore. Chi ha gli interrupt spenti *ritarda* soltanto, che Linux
   dichiara corretto (`mm/mmu_gather.c:245-248`). Precedenti: FreeBSD `mp_machdep.c:558-569,700-706`
   (ha spinlock IRQ-off come noi, e mette il vincolo sull'INIZIATORE: `KASSERT` a :668-670),
   OSv `arch/x64/mmu.cc:63-68`, Linux `kernel/smp.c:920-921` ("must be fast and non-blocking").

**Conseguenza per le nostre patch**: con il modello ad ack, #1 e #2 diventerebbero superflue.
Restano valide come rimedio locale finche' il modello e' quello attuale, e la #4 (inversione
node lock / pt_lock) vale a prescindere.

**Il test c'e' gia'**: `tmpfs_punch_nosync` con **VCPUS=4** — master passa 32 round due volte
su due, il nostro albero si pianta (due osservazioni). A VCPUS=2 non si vede.

## Aperto

- Ablazioni #5 e #6 (in corso quando questa nota è stata scritta): log in `/tmp/soaklab/y-*.log`.
- Soak da 30 min **senza** le fix dei lock, per avere il "prima" moderno accanto ai 10,3 GB del "dopo".
- Due candidate in più, dall'indagine sui contesti, **non ancora scritte come patch**:
  - `tmpfsfile_get_blocks` (`klib/tmpfs.c:114-121`) cammina la rangemap `dirty` **senza il lock**
    mentre write inserisce e punch rimuove sotto lock. **Riprodotto sotto KVM**: page fault in
    `rbnode_get_next` da `fstat`.
  - `pagecache_commit_dirty_pages` tiene `pc->global_lock` attraverso una catena che dorme.
    **Preesistente in master, identico.**
- Il difetto di ciclo di vita dei contesti esiste ma **non è raggiungibile** dal carico reale:
  `context_acquire` distingue solo `active_cpu`, non "parcheggiato" da "sospeso". Forzandolo,
  `assert(remain-- > 0)` scatta 2/2. Merita una PR sua, non dentro questa serie.

## Trappole in cui sono caduto (costano ore)

- `cd` dentro una catena di comandi non ha l'effetto che sembra: un `git checkout` è finito nel
  repo dell'utente e ha staccato HEAD. **Usare sempre `git -C` e `make -C`.**
- `pkill -f <pattern>` uccide anche la shell che lo esegue, se il pattern compare nella sua riga
  di comando. E `pgrep -x qemu-system-x86_64` non trova nulla: il nome del processo è troncato a
  15 caratteri (`qemu-system-x86`).
- `git clone` prende solo ciò che è **committato**: un test registrato ma non committato non
  arriva nel clone, e l'ablazione fallisce con "Nessuna regola per generare".
- `make kernel` non ricompila le klib se il loro `.c` non è cambiato: dopo aver toccato
  `src/fs/fs.h` serve `touch klib/*.c`.
- I blocchi di `st_blocks` su tmpfs contano solo dopo un **commit** del pagecache: senza `fsync`
  il conto è zero su qualunque kernel, e un test che lo ignora passa per il motivo sbagliato.

## Dove sono le cose

- Banco di prova, log, configurazioni JVM: `/tmp/soaklab/`
- Clone di master vergine con i test registrati: `/tmp/vergine`
- Programma di carico JVM: `/tmp/soaklab/GcChurn.java` (estratto da `.github/workflows/verifyZgcArm64.yml`)
- Pacchetto JRE usato nel guest: `AngeloRubens/AzulJREx64Linux:25.0.1` (x64),
  `AngeloRubens/TemurinJREarm64Linux:25.0.1` (arm64)
