.. SPDX-License-Identifier: GPL-2.0

==================================
CXL Hotness Monitoring Unit Driver
==================================

CXL r3.2 introduced the CXL Hotness Monitoring Unit (CHMU). A CHMU allows
software running on a CXL Host to identify hot memory ranges, that is those with
higher access frequency relative to other memory ranges.

A given Logical Device (presentation of a CXL memory device seen by a particular
host) can provide 1 or more CHMU each of which supports 1 or more separately
programmable CHMU Instances (CHMUI). These CHMUI are mostly independent with
the exception that there can be restrictions on them tracking the same memory
regions. The CHMUs are always completely independent.
The naming of the units is cxl_hmu_memX.Y.Z where memX matches the naming
of the memory device in /sys/bus/cxl/devices/, Y is the CHMU index and
Z is the CHMUI index with the CHMU.

Each CHMUI provides a ring buffer structure known as the Hot List from which the
host can read back entries that describe the hotness of a particular region of
memory (Hot List Units). The Hot List Unit combines a Unit Address and an access
count for the particular address. Unit address to DPA requires multiplication
by the unit size. Thus, for large unit sizes the device may support higher
counts. It is these Hot List Units that the driver provides via a perf AUX
buffer by copying them from PCI BAR space.

The unit size at which hotness is measured is configurable for each CHMUI and
all measurement is done in Device Physical Address space. To relate this to
Host Physical Address space the HDM (Host-Managed Device Memory) decoder
configuration must be taken into account to reflect the placement in a
CXL Fixed Memory Window and any interleaving.

The CHMUI can support interrupts on fills above a watermark, or on overflow
of the hotlist.

A CHMUI can support two different basic modes of operation. Epoch and
Always On. These affect what is placed on the hotlist. Note that the actual
implementation of tracking is implementation defined and likely to be
inherently imprecise in that the hottest pages may not be discovered due to
resource exhaustion and the hotness counts may not represent accurately how
hot they are. The specification allows for a very high degree of flexibility
in implementation, important as it is likely that a number of different
hardware implementations will be chosen to suit particular silicon and accuracy
budgets.

Operation and configuration
===========================

Each CHMUI is a device under /sys/bus/cxl/devices/hmu_memX.Y/hmu_memX.Y.Z.
The PMU exists for every instance with the fill-threshold interrupt and a
vector to deliver it. An instance whose Interrupt Message Number has no
vector is still registered, with the warning "no vector for interrupt
message number N (-errno): no interrupt, no PMU", and so is one whose vector
the driver could not request ("interrupt N not available (-errno): no PMU").

An instance has one user at a time. An open perf event holds the instance
from perf_event_open() until the event is closed, whether or not it is
enabled, so a second event on the same PMU, or an event on an instance that
another user holds, fails to open with -EBUSY. So does an event whose range
overlaps that of a held sibling instance when the instances do not support
Address Overlap. Opening an event needs CAP_PERFMON, since the hotlist
carries raw device physical addresses.

The device reports the width of its counter only once the unit size is
programmed, so opening an event programs the event's unit size into the
instance, which it holds by then and which is not enabled. A
hotness_threshold that does not fit the counter fails the open with -EINVAL,
and a device that reports no usable width fails it with -EIO.

An event that is open can still fail to start, when the device does not
follow: it does not come out of a counter reset or an enable in time, or
shows no counter width. The kernel logs "start failed (-errno)" once, and
the event then stays open and records nothing until it is closed. The
device is not tried again for it; open a new event.

An example command line is below. The event counts for the whole device,
not for a task or a cgroup: it is opened system-wide, as with -a. A
per-task event (a command without -a) fails with -EOPNOTSUPP and a cgroup's
event (-G) with -EINVAL. The PMU is served by one CPU, the one in its
"cpumask" attribute, which is what perf opens the event on; the kernel puts
the event there whatever CPU it was asked for::

  $perf record -a  -e cxl_hmu_mem0.0.0/epoch_type=0,access_type=6,\
  hotness_threshold=1024,epoch_multiplier=4,epoch_scale=4,range_base=0,\
  range_size=4,randomized_downsampling=0,hotness_granual=12/ -- sleep 10

  $perf report --dump-raw-trace

which will produce a list of hotlist entries, one per line with a short header
to provide sufficient information to interpret the entries::

  . ... CXL_HMU data: size 33512 bytes
  Header 0: units: 29c counter_width 10
  Header 1 : deadbeef
  0000000000000283
  0000000000010364
  0000000000020366
  000000000003033c
  0000000000040343
  00000000000502ff
  000000000006030d
  000000000007031a
  ...

The least significant counter_width bits (here 16, hex 10) are the counter
value, all higher bits are the unit index.  Multiply by the unit size
to get a Device Physical Address.

The AUX data is a sequence of records, each a 16 byte header followed by
8 byte entries: bits 15:0 of the first header word are the number of
entries in the record, bits 23:16 the counter width, and the second word is
the magic 0xdeadbeef. Each hotlist the driver drains, on an interrupt or
when the event stops, is written as one record in its own AUX output. A
decoder reads record by record, as perf report does above, rather than
assuming one header per AUX output.

The two ways of losing data have a flag each. A hotlist overflow reported
by a device that loses hot units for it marks the AUX record as partial
(PERF_AUX_FLAG_PARTIAL): what was read is sound, but units that became hot
while the list was full are missing. A device that supports Postponed on
Overflow inserts those units later, loses none, and its records are not
marked.

The free space of the AUX buffer is looked up at every delivery, and only
what fits is taken from the device. If entries are left behind for want of
room the record is marked truncated (PERF_AUX_FLAG_TRUNCATED) and perf
disables the event, as for any AUX PMU whose reader fell behind. Stopping
the event writes what the reader has made room for by then; the rest is
discarded. A larger -m,,N buffer or a longer epoch keeps up.

A parameter left at 0 takes a default: the smallest unit size and down
sampling factor the device supports, its shortest epoch when epoch_scale
and epoch_multiplier are both 0 (one of them alone at 0 is -EINVAL), and
the whole device as the range. access_type has no default and must be one
the device supports. The bits of config that none of the parameters covers
must be 0, else the event fails to open with -EINVAL.

The parameters are as follows:

epoch_type
----------

Two values may be supported::

  0 - Epoch based operation
  1 - Always on operation


0. Epoch Based Operation
~~~~~~~~~~~~~~~~~~~~~~~~

An Epoch is a period of time after which a counter is assessed for hotness.

The device may have a global sense of an Epoch but it may also operate them on
a per counter, or per region of device basis. This is a function of the
implementation and is not controllable, but is discoverable. In a global Epoch
scheme at start of each Epoch all counters are zeroed / deallocated. Counters
are then allocated in a hardware specific manner and accesses counted. At the
completion of the Epoch the counters are compared with a threshold and entries
with a count above a configurable threshold are added to the hotlist. A new
Epoch is then begun with all counters cleared.

In non-global Epoch scheme, when the Epoch of a given counter begins is not
specified. An example might be an Epoch for counter only starting on first
touch to the relevant memory region.  When a local Epoch ends the counter is
compared to the threshold and if appropriate added to the hotlist.

Note, in Epoch Based Operation, the counter in the hotlist entry provides
information on how hot the memory is as the counter for the full Epoch is
provided.

1. Always on Operation
~~~~~~~~~~~~~~~~~~~~~~

In this mode, counters may all be reset before enabling the CHMUI. Then
counters are allocated to particular memory units via an hardware specific
method, perhaps on first touch.  When a counter passes the configurable
hotness threshold an entry is added to the hotlist and that counter is freed
for reuse.

In this scheme the count provided in the hotlist entry is not useful as it will
depend only on the configured threshold.

access_type
-----------

The parameter controls which access are counted::

  1 - Non-TEE read only
  2 - Non-TEE write only
  3 - Non-TEE read and write
  4 - TEE and Non-TEE read only
  5 - TEE and Non-TEE write only
  6 - TEE and Non-tee read and write


TEE here refers to a trusted execution environment, specifically one that
results in the T bit being set in the CXL transactions.


hotness_granual
---------------

Unit size at which tracking is performed.  Must be at least 256 bytes but
hardware may only support some sizes. Expressed as a power of 2. e.g. 12 = 4kiB.
0 selects the smallest size the device supports.

hotness_threshold
-----------------

This is the minimum counter value that must be reached for the unit to count as
hot and be added to the hotlist.

The possible range may be dependent on the unit size as a larger unit size
requires more bits on the hotlist entry leaving fewer available for the hotness
counter.

epoch_multiplier and epoch_scale
--------------------------------

The length of an epoch (in epoch mode) is controlled by these two parameters
with the decoded epoch_scale multiplied by the epoch_multiplier to give the
overall epoch length.

epoch_scale::

  1 - 100 usecs
  2 - 1 msec
  3 - 10 msecs
  4 - 100 msecs
  5 - 1 second

range_base and range_size
-------------------------

Each CHMUI has a
bitmap that controls what Device Physical Address spaces is tracked. Each bit
represents 256MiB of DPA space.

This interface provides a simple base and size in units of 256MiB to configure
this bitmap. The bits in the specified range are set and every other bit that
covers the device's capacity is cleared; base + size may not exceed that
capacity. A size of 0 selects the whole device.

downsampling_factor
-------------------

Hardware may be incapable of counting accesses at full speed or it may be
desirable to count over a longer period during which the counters would
overflow.  This control allows selection of a down sampling factor expressed
as a power of 2 between 1 and 32768, given as the exponent (0 for 1, 1 for 2,
and so on).  Default is minimum supported downsampling factor.

randomized_downsampling
-----------------------

To avoid problems with downsampling when accesses are periodic this option
allows for an implementation defined randomization of the sampling interval,
whilst remaining close to the specified downsampling_factor.

Trace events
============

The cxl_hmu trace system has one event, which starts with the name of the
instance device.

cxl_hmu_hotlist
  One read of the hotlist, by the PMU or by an in-kernel user, including a
  read that found it empty: nr is the number of entries taken,
  counter_width the counter width of the session, and overflowed is 1 if the
  device had reported a hotlist overflow since the previous read.
