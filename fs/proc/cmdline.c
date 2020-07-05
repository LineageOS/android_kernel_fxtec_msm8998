#include <asm/setup.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#define BOOT_REASON_PON 6

static char proc_command_line[COMMAND_LINE_SIZE];

static int cmdline_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", proc_command_line);
	return 0;
}

static int cmdline_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cmdline_proc_show, NULL);
}

static const struct file_operations cmdline_proc_fops = {
	.open		= cmdline_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void append_boot_mode(void)
{
	if (boot_reason == BOOT_REASON_PON) {
		strlcat(proc_command_line, " androidboot.mode=charger",
				COMMAND_LINE_SIZE);
	} else {
		strlcat(proc_command_line, " androidboot.mode=unknown",
				COMMAND_LINE_SIZE);
	}
}

static int __init proc_cmdline_init(void)
{
	strncpy(proc_command_line, saved_command_line, COMMAND_LINE_SIZE);

	append_boot_mode();

	proc_create("cmdline", 0, NULL, &cmdline_proc_fops);
	return 0;
}
fs_initcall(proc_cmdline_init);
