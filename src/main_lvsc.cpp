#include <cstdio>
#include <csignal>
#include <cstdarg>
#include <cstring>
#include <unistd.h>
#include <functional>
#include <memory>
#include <string>
#include <libvirt/libvirt.h>
#include <vpx/vpx_encoder.h>
#include <vpx/vp8cx.h>

using namespace std;

// A flag to store whether we are in the main record it and a signal handler to update it.
static volatile sig_atomic_t capturing = 1;

static void signal_handler(int sig, siginfo_t *si, void *unused)
{
    capturing = 0;
}

// Maths and endian routines.
uint8_t clamp(ssize_t x)
{
    return max(min(x, ssize_t(255)), ssize_t(0));
}

static void mem_put_le16(void *vmem, int val)
{
    uint8_t *mem = (uint8_t *)vmem;

    mem[0] = (uint8_t)((val >> 0) & 0xff);
    mem[1] = (uint8_t)((val >> 8) & 0xff);
}

static void mem_put_le32(void *vmem, int val)
{
    uint8_t *mem = (uint8_t *)vmem;

    mem[0] = (uint8_t)((val >> 0) & 0xff);
    mem[1] = (uint8_t)((val >> 8) & 0xff);
    mem[2] = (uint8_t)((val >> 16) & 0xff);
    mem[3] = (uint8_t)((val >> 24) & 0xff);
}

// A buffer to hold the screenshot of about 24MB, the image MUST fit in this.
const auto SIZE = 1024 * 768 * 3 * 10;
uint8_t buffer[SIZE];

// VP9 info
static const int VP9_FOURCC = 0x30395056;

// Statically chosen FPS setting
static const int FPS_NUMERATOR = 1;
static const int FPS_DENOMINATOR = 5;
static const int KEYFRAME_INTERVAL = 10;

// Output functions
bool DEBUG = false;

void perform_debug(const char *fmt, ...)
{
    if (DEBUG)
    {
        va_list ap;
        va_start(ap, fmt);
        vfprintf(stderr, fmt, ap);
        va_end(ap);
    }
}

#define XSTRINGIFY(x) #x
#define STRINGIFY(x) XSTRINGIFY(x)
#define debug(...) perform_debug("[" __FILE__ ":" STRINGIFY(__LINE__) "]: " __VA_ARGS__)

void fatal(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    exit(EXIT_FAILURE);
}

void output(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stdout, fmt, ap);
    va_end(ap);
}

void usage_exit(const char *name)
{
    fatal(
        "Usage: %s <domain> <outfile> [--connect <connection_uri>] [--debug]\n"
        "Takes screenshots of a domain and combines them into a WEBM file.\n"
        "Can only manage about 2 fps and the output is sped up to 5 fps.\n",
        name);
}

// A writer that can take in RGB 24 bit frames and save them out as a IVF VPX stream.
struct IVFVPX9Writer
{
    FILE *outfile;
    int width;
    int height;
    int frames_written;
    int frames_encoded;
    vpx_codec_ctx_t codec;
    vpx_image_t img;

    void write_file_header()
    {
        debug("Writing file header\n");
        char header[32];
        header[0] = 'D';
        header[1] = 'K';
        header[2] = 'I';
        header[3] = 'F';
        mem_put_le16(header + 4, 0);                // version
        mem_put_le16(header + 6, 32);               // header size
        mem_put_le32(header + 8, VP9_FOURCC);       // fourcc
        mem_put_le16(header + 12, width);           // width
        mem_put_le16(header + 14, height);          // height
        mem_put_le32(header + 16, FPS_DENOMINATOR); // rate
        mem_put_le32(header + 20, FPS_NUMERATOR);   // scale
        mem_put_le32(header + 24, frames_written);  // length
        mem_put_le32(header + 28, 0);               // unused
        auto position = ftell(outfile);
        rewind(outfile);
        fwrite(header, 1, 32, outfile);
        if (position >= 32)
            fseek(outfile, position, SEEK_SET);
    }

    void write_ivf_frame_header(FILE *outfile, int64_t pts, uint32_t frame_size)
    {
        debug("Writing frame header\n");

        char header[12];

        mem_put_le32(header, (int)frame_size);
        mem_put_le32(header + 4, (int)(pts & 0xFFFFFFFF));
        mem_put_le32(header + 8, (int)(pts >> 32));
        fwrite(header, 1, 12, outfile);
    }

    int vpx_video_writer_write_frame(const uint8_t *buffer, size_t size, int64_t pts)
    {
        debug("Writing frame\n");
        write_ivf_frame_header(outfile, pts, size);
        if (fwrite(buffer, 1, size, outfile) != size)
            return 0;
        ++frames_written;
        return 1;
    }

    int encode_frame(bool flush = false)
    {
        debug("Encoding frame with flush=%d\n", flush);
        auto frame_index = !flush ? frames_encoded : -1;
        int flags = frame_index % KEYFRAME_INTERVAL == 0 ? VPX_EFLAG_FORCE_KF : 0;
        auto img = !flush ? &this->img : nullptr;
        if (!flush)
            ++frames_encoded;
        int got_pkts = 0;
        vpx_codec_iter_t iter = NULL;
        const vpx_codec_cx_pkt_t *pkt = NULL;
        const vpx_codec_err_t res =
            vpx_codec_encode(&codec, img, frame_index, 1, flags, VPX_DL_GOOD_QUALITY);
        if (res != VPX_CODEC_OK)
            fatal("Failed to encode frame. %s\n", vpx_codec_error_detail(&codec));
        while ((pkt = vpx_codec_get_cx_data(&codec, &iter)) != NULL)
        {
            got_pkts = 1;
            if (pkt->kind == VPX_CODEC_CX_FRAME_PKT)
            {
                // const int keyframe = (pkt->data.frame.flags & VPX_FRAME_IS_KEY) != 0;
                if (!vpx_video_writer_write_frame((const uint8_t *)pkt->data.frame.buf,
                                                  pkt->data.frame.sz,
                                                  pkt->data.frame.pts))
                {
                    fatal("Failed to write compressed frame. %s\n", vpx_codec_error_detail(&codec));
                }
                fflush(stdout);
            }
        }
        return got_pkts;
    }

    void update_image(uint8_t *buffer)
    {
        debug("Updating image\n");
        size_t dst_pos = 0;
        uint8_t *dest = img.planes[0];
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                int o = (y * width + x) * 3;

                ssize_t r = buffer[o];
                ssize_t g = buffer[o + 1];
                ssize_t b = buffer[o + 2];

                ssize_t y = (66 * r + 129 * g + 25 * b + 128) / 256 + 16;
                dest[dst_pos++] = clamp(y);
            }
        }

        dst_pos = 0;
        dest = img.planes[1];
        for (int y = 0; y < height; y += 2)
        {
            for (int x = 0; x < width; x += 2)
            {
                int o = (y * width + x) * 3;

                ssize_t r = buffer[o];
                ssize_t g = buffer[o + 1];
                ssize_t b = buffer[o + 2];

                ssize_t u = (-38 * r - 74 * g + 112 * b + 128) / 256 + 128;
                dest[dst_pos++] = clamp(u);
            }
        }

        dst_pos = 0;
        dest = img.planes[2];
        for (int y = 0; y < height; y += 2)
        {
            for (int x = 0; x < width; x += 2)
            {
                int o = (y * width + x) * 3;

                ssize_t r = buffer[o];
                ssize_t g = buffer[o + 1];
                ssize_t b = buffer[o + 2];

                ssize_t v = (112 * r - 94 * g - 18 * b + 128) / 256 + 128;
                dest[dst_pos++] = clamp(v);
            }
        }
    }

    void flush()
    {
        debug("Flushing writer\n");

        write_file_header();
        // Flush encoder.
        while (encode_frame(true))
        {
        }
        fflush(outfile);
    }

    ~IVFVPX9Writer()
    {
        debug("Destroying writer\n");

        flush();
        fclose(outfile);
        vpx_img_free(&img);
        vpx_codec_destroy(&codec);
    }

    IVFVPX9Writer(const char *filename, int width, int height) : outfile(fopen(filename, "w")), width(width), height(height), frames_written(0), frames_encoded(0), codec(vpx_codec_ctx_t()), img(vpx_image_t())
    {
        debug("Creating writer for %s of size %dx%d\n", filename, width, height);

        vpx_codec_enc_cfg_t cfg;
        auto error = vpx_codec_enc_config_default(vpx_codec_vp9_cx(), &cfg, 0);
        if (error)
            fatal("Failed to get default codec config. %s\n", vpx_codec_error_detail(&codec));

        cfg.g_w = width;
        cfg.g_h = height;
        cfg.g_timebase.num = FPS_NUMERATOR;
        cfg.g_timebase.den = FPS_DENOMINATOR;
        cfg.g_error_resilient = 0;

        if (vpx_codec_enc_init(&codec, vpx_codec_vp9_cx(), &cfg, 0))
            fatal("Failed to initialize encoder with VP9 codec. %s\n", vpx_codec_error_detail(&codec));

        if (vpx_codec_control_(&codec, VP9E_SET_LOSSLESS, 1))
            fatal("Failed to use lossless mode on VP9 codec. %s\n", vpx_codec_error_detail(&codec));

        write_file_header();
        vpx_img_alloc(&img, VPX_IMG_FMT_I420, width, height, 1);
    }

    IVFVPX9Writer(const IVFVPX9Writer &o) = delete;
};

int main(int argc, char **argv)
{
    // Argument processing
    string domain_name;
    string output_file;
    string connection_uri = "qemu:///system";

    const string connection_option = "--connection";
    const string debug_option = "--debug";

    for (int i = 1; i < argc; ++i)
    {
        string arg = argv[i];
        if (arg.substr(0, connection_option.size()) == connection_option)
        {
            connection_uri = argv[++i];
        }
        else if (arg.substr(0, debug_option.size()) == debug_option)
        {
            DEBUG = true;
        }
        else if (domain_name.empty())
        {
            domain_name = arg;
        }
        else if (output_file.empty())
        {
            output_file = arg;
        }
        else
        {
            usage_exit(argv[0]);
        }
    }
    if (domain_name.empty() || output_file.empty())
    {
        usage_exit(argv[0]);
    }

    string tmp_file = "/tmp/" + domain_name + ".webm";
    unique_ptr<IVFVPX9Writer> video_stream;

    // Set up the signal handler
    struct sigaction sigact;
    sigact.sa_sigaction = signal_handler;
    sigact.sa_flags = SA_RESTART | SA_SIGINFO;
    sigaction(SIGINT, &sigact, (struct sigaction *)NULL);

    // Initialize libVirt and set up some wrappers for the libvirt pointers
    virInitialize();

    typedef unique_ptr<virConnect, function<void(virConnectPtr)>> Connection;
    auto connect = [&](string name) -> auto
    {
        auto deleter = [&](virConnectPtr ptr) {
            virConnectClose(ptr);
        };
        auto connection = Connection(virConnectOpen(name.c_str()), deleter);
        if (!connection)
            fatal("Could not connect to %s\n", name.c_str());
        return connection;
    };

    typedef unique_ptr<virDomain, function<void(virDomainPtr)>> Domain;
    auto get_domain = [&](Connection & connection, string name) -> auto
    {
        auto deleter = [&](virDomainPtr ptr) {
            virDomainFree(ptr);
        };
        auto domain = Domain(virDomainLookupByName(connection.get(), name.c_str()), deleter);
        if (!domain)
            fatal("Could not find domain %s\n", name.c_str());
        if (virDomainIsActive(domain.get()) != 1)
            fatal("Domain must be running\n");
        return domain;
    };

    typedef unique_ptr<virStream, function<void(virStreamPtr)>> Stream;
    auto new_stream = [&](Connection & connection) -> auto
    {
        auto deleter = [&](virStreamPtr ptr) {
            virStreamFree(ptr);
        };
        return Stream(virStreamNew(connection.get(), 0), deleter);
    };

    auto take_screenshot = [&](Domain &domain, Stream &stream, uint8_t *buffer, size_t len) -> ssize_t {
        auto mimetype = virDomainScreenshot(domain.get(), stream.get(), 0, 0);
        if (!mimetype)
            return -1;
        free(mimetype);

        auto base = buffer;
        auto remaining_size = SIZE;

        while (true)
        {
            auto res = virStreamRecv(stream.get(), (char *)base, remaining_size);
            if (res < 0)
            {
                virStreamAbort(stream.get());
                return -1;
            }
            else if (res == 0)
            {
                virStreamFinish(stream.get());
                return base - buffer;
            }
            base += res;
            remaining_size -= res;
        }
    };

    auto connection = connect(connection_uri.c_str());
    auto domain = get_domain(connection, domain_name);
    auto stream = new_stream(connection);

    output("Starting capture. Press Ctrl+C or send SIGINT to end recording\n");
    while (capturing)
    {
        if (take_screenshot(domain, stream, buffer, SIZE) < 0)
            continue;

        int pwidth, pheight, pdepth, chars;
        sscanf((char *)buffer, "P6 %d %d %d%n", &pwidth, &pheight, &pdepth, &chars);
        ++chars;
        if (!video_stream)
            video_stream = make_unique<IVFVPX9Writer>((tmp_file).c_str(), pwidth, pheight);
        video_stream->update_image(buffer + chars);
        video_stream->encode_frame();
    }
    output("Ending capture. %d frames captured. Flushing streams and combining into webm\n", video_stream->frames_encoded);

    if (video_stream)
        video_stream->flush();

    string merge_system_str = "mkvmerge -o ";
    merge_system_str += output_file;
    merge_system_str += " -w ";
    merge_system_str += tmp_file;
    debug("Shelling out to mkvmerge to create webm container: %s\n", merge_system_str.c_str());
    if (system(merge_system_str.c_str()))
        fatal("Failed to launch %s, check that mkvmerge is intalled.", merge_system_str.c_str());
    if (unlink(tmp_file.c_str()) != 0)
    {
        output("Failed to remove temporary file %s\n", tmp_file.c_str());
    }

    return 0;
}