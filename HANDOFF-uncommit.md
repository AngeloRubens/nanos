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
