from http import HTTPStatus
from typing import Any

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.chassis_ultrasonic_response_200 import ChassisUltrasonicResponse200
from ...models.error_envelope import ErrorEnvelope
from ...types import Response


def _get_kwargs() -> dict[str, Any]:

    _kwargs: dict[str, Any] = {
        "method": "get",
        "url": "/v1/chassis/ultrasonic",
    }

    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ChassisUltrasonicResponse200 | ErrorEnvelope | None:
    if response.status_code == 200:
        response_200 = ChassisUltrasonicResponse200.from_dict(response.json())

        return response_200

    if response.status_code == 400:
        response_400 = ErrorEnvelope.from_dict(response.json())

        return response_400

    if response.status_code == 404:
        response_404 = ErrorEnvelope.from_dict(response.json())

        return response_404

    if response.status_code == 409:
        response_409 = ErrorEnvelope.from_dict(response.json())

        return response_409

    if response.status_code == 502:
        response_502 = ErrorEnvelope.from_dict(response.json())

        return response_502

    if response.status_code == 504:
        response_504 = ErrorEnvelope.from_dict(response.json())

        return response_504

    if client.raise_on_unexpected_status:
        raise errors.UnexpectedStatus(response.status_code, response.content)
    else:
        return None


def _build_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> Response[ChassisUltrasonicResponse200 | ErrorEnvelope]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    *,
    client: AuthenticatedClient | Client,
) -> Response[ChassisUltrasonicResponse200 | ErrorEnvelope]:
    """Read the mobile base's ultrasonic proximity ring

     A HARDWARE-CHECK reading: the daemon does not brake, stop or re-route the base on it and it is NOT a
    safety interlock. The ring is carried only by the vendor's TCP sensor push service, which one
    mobile-base firmware generation serves and the other does not, so `supported` false with a stated
    `reason` is a normal answer about the base rather than an error. `supported` true with `source`
    `none` is the other case: this base does serve the ring, and this read did not get one -- `reason`
    says why. A channel that received no echo is reported with `valid` false and `range_m` null. The
    vendor's rule for that is ONE test, `range >= max_range`, and the out-of-range number itself is not
    a second fact: the D1 mobile base measured on 2026-09-19 sends 65.533 on three channels and 100.0 on
    a fourth against a `max_range` of 4.5, and both fail the one test. The number stays visible on
    `raw_range_m`, must never be read as a distance, and must not be used to sort channels into kinds.
    `age_ms` is per channel as well as per reading, because the base does not always send every
    configured channel. `link` says how the daemon is currently reading the base: `cycle` for a short
    connection per read, `held` while a consumer of the stream operation keeps it open.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisUltrasonicResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs()

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    *,
    client: AuthenticatedClient | Client,
) -> ChassisUltrasonicResponse200 | ErrorEnvelope | None:
    """Read the mobile base's ultrasonic proximity ring

     A HARDWARE-CHECK reading: the daemon does not brake, stop or re-route the base on it and it is NOT a
    safety interlock. The ring is carried only by the vendor's TCP sensor push service, which one
    mobile-base firmware generation serves and the other does not, so `supported` false with a stated
    `reason` is a normal answer about the base rather than an error. `supported` true with `source`
    `none` is the other case: this base does serve the ring, and this read did not get one -- `reason`
    says why. A channel that received no echo is reported with `valid` false and `range_m` null. The
    vendor's rule for that is ONE test, `range >= max_range`, and the out-of-range number itself is not
    a second fact: the D1 mobile base measured on 2026-09-19 sends 65.533 on three channels and 100.0 on
    a fourth against a `max_range` of 4.5, and both fail the one test. The number stays visible on
    `raw_range_m`, must never be read as a distance, and must not be used to sort channels into kinds.
    `age_ms` is per channel as well as per reading, because the base does not always send every
    configured channel. `link` says how the daemon is currently reading the base: `cycle` for a short
    connection per read, `held` while a consumer of the stream operation keeps it open.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisUltrasonicResponse200 | ErrorEnvelope
    """

    return sync_detailed(
        client=client,
    ).parsed


async def asyncio_detailed(
    *,
    client: AuthenticatedClient | Client,
) -> Response[ChassisUltrasonicResponse200 | ErrorEnvelope]:
    """Read the mobile base's ultrasonic proximity ring

     A HARDWARE-CHECK reading: the daemon does not brake, stop or re-route the base on it and it is NOT a
    safety interlock. The ring is carried only by the vendor's TCP sensor push service, which one
    mobile-base firmware generation serves and the other does not, so `supported` false with a stated
    `reason` is a normal answer about the base rather than an error. `supported` true with `source`
    `none` is the other case: this base does serve the ring, and this read did not get one -- `reason`
    says why. A channel that received no echo is reported with `valid` false and `range_m` null. The
    vendor's rule for that is ONE test, `range >= max_range`, and the out-of-range number itself is not
    a second fact: the D1 mobile base measured on 2026-09-19 sends 65.533 on three channels and 100.0 on
    a fourth against a `max_range` of 4.5, and both fail the one test. The number stays visible on
    `raw_range_m`, must never be read as a distance, and must not be used to sort channels into kinds.
    `age_ms` is per channel as well as per reading, because the base does not always send every
    configured channel. `link` says how the daemon is currently reading the base: `cycle` for a short
    connection per read, `held` while a consumer of the stream operation keeps it open.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisUltrasonicResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs()

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    *,
    client: AuthenticatedClient | Client,
) -> ChassisUltrasonicResponse200 | ErrorEnvelope | None:
    """Read the mobile base's ultrasonic proximity ring

     A HARDWARE-CHECK reading: the daemon does not brake, stop or re-route the base on it and it is NOT a
    safety interlock. The ring is carried only by the vendor's TCP sensor push service, which one
    mobile-base firmware generation serves and the other does not, so `supported` false with a stated
    `reason` is a normal answer about the base rather than an error. `supported` true with `source`
    `none` is the other case: this base does serve the ring, and this read did not get one -- `reason`
    says why. A channel that received no echo is reported with `valid` false and `range_m` null. The
    vendor's rule for that is ONE test, `range >= max_range`, and the out-of-range number itself is not
    a second fact: the D1 mobile base measured on 2026-09-19 sends 65.533 on three channels and 100.0 on
    a fourth against a `max_range` of 4.5, and both fail the one test. The number stays visible on
    `raw_range_m`, must never be read as a distance, and must not be used to sort channels into kinds.
    `age_ms` is per channel as well as per reading, because the base does not always send every
    configured channel. `link` says how the daemon is currently reading the base: `cycle` for a short
    connection per read, `held` while a consumer of the stream operation keeps it open.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisUltrasonicResponse200 | ErrorEnvelope
    """

    return (
        await asyncio_detailed(
            client=client,
        )
    ).parsed
